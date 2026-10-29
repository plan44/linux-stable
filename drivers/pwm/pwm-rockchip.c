// SPDX-License-Identifier: GPL-2.0-only
/*
 * PWM driver for Rockchip SoCs
 *
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
 * Copyright (C) 2014 ROCKCHIP, Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/overflow.h>
#include <linux/pinctrl/consumer.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/time.h>

#define PWM_CTRL_TIMER_EN	(1 << 0)
#define PWM_CTRL_OUTPUT_EN	(1 << 3)

#define PWM_ENABLE		(1 << 0)
#define PWM_CONTINUOUS		(1 << 1)
#define PWM_DUTY_POSITIVE	(1 << 3)
#define PWM_DUTY_NEGATIVE	(0 << 3)
#define PWM_INACTIVE_NEGATIVE	(0 << 4)
#define PWM_INACTIVE_POSITIVE	(1 << 4)
#define PWM_POLARITY_MASK	(PWM_DUTY_POSITIVE | PWM_INACTIVE_POSITIVE)
#define PWM_OUTPUT_LEFT		(0 << 5)
#define PWM_LOCK_EN		(1 << 6)
#define PWM_LP_DISABLE		(0 << 8)

/* PWM v4 uses high-word write masks for control registers. */
#define PWM_V4_VERSION		0x00
#define PWM_V4_ENABLE		0x04
#define PWM_V4_CLK_CTRL		0x08
#define PWM_V4_CTRL		0x0c
#define PWM_V4_PERIOD		0x10
#define PWM_V4_DUTY		0x14
#define PWM_V4_OFFSET		0x18
#define PWM_V4_RPT		0x1c
#define PWM_V4_INT_EN		0x74
#define PWM_V4_ENABLE_MASK	GENMASK(1, 0)
#define PWM_V4_UPDATE		BIT(2)
#define PWM_V4_GLOBAL_JOIN	BIT(4)
#define PWM_V4_DUTY_POSITIVE	BIT(2)
#define PWM_V4_INACTIVE_POSITIVE	BIT(3)
#define PWM_V4_POLARITY_MASK	GENMASK(3, 2)
#define PWM_V4_CONTINUOUS	1
#define PWM_V4_MODE_MASK		GENMASK(1, 0)
#define PWM_V4_ALIGNED_INVALID	BIT(5)
#define PWM_V4_CLOCK_MASK	(GENMASK(2, 0) | GENMASK(15, 4))

struct rockchip_pwm_chip {
	struct clk *clk;
	struct clk *pclk;
	bool v4_enabled;
	const struct rockchip_pwm_data *data;
	void __iomem *base;
};

struct rockchip_pwm_regs {
	unsigned long duty;
	unsigned long period;
	unsigned long cntr;
	unsigned long ctrl;
};

struct rockchip_pwm_data {
	struct rockchip_pwm_regs regs;
	unsigned int prescaler;
	bool supports_polarity;
	bool supports_lock;
	bool v4;
	u32 enable_conf;
};

static inline struct rockchip_pwm_chip *to_rockchip_pwm_chip(struct pwm_chip *chip)
{
	return pwmchip_get_drvdata(chip);
}

static int rockchip_pwm_get_state(struct pwm_chip *chip,
				  struct pwm_device *pwm,
				  struct pwm_state *state)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	u32 enable_conf = pc->data->enable_conf;
	unsigned long clk_rate;
	u64 tmp;
	u32 val;
	int ret;

	ret = clk_enable(pc->pclk);
	if (ret)
		return ret;

	ret = clk_enable(pc->clk);
	if (ret) {
		clk_disable(pc->pclk);
		return ret;
	}

	clk_rate = clk_get_rate(pc->clk);

	tmp = readl_relaxed(pc->base + pc->data->regs.period);
	tmp *= pc->data->prescaler * NSEC_PER_SEC;
	state->period = DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);

	tmp = readl_relaxed(pc->base + pc->data->regs.duty);
	tmp *= pc->data->prescaler * NSEC_PER_SEC;
	state->duty_cycle =  DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);

	val = readl_relaxed(pc->base + pc->data->regs.ctrl);
	state->enabled = (val & enable_conf) == enable_conf;

	if (pc->data->supports_polarity && !(val & PWM_DUTY_POSITIVE))
		state->polarity = PWM_POLARITY_INVERSED;
	else
		state->polarity = PWM_POLARITY_NORMAL;

	clk_disable(pc->clk);
	clk_disable(pc->pclk);

	return 0;
}

static void rockchip_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			       const struct pwm_state *state)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	unsigned long period, duty;
	u64 clk_rate, div;
	u32 ctrl;

	clk_rate = clk_get_rate(pc->clk);

	/*
	 * Since period and duty cycle registers have a width of 32
	 * bits, every possible input period can be obtained using the
	 * default prescaler value for all practical clock rate values.
	 */
	div = clk_rate * state->period;
	period = DIV_ROUND_CLOSEST_ULL(div,
				       pc->data->prescaler * NSEC_PER_SEC);

	div = clk_rate * state->duty_cycle;
	duty = DIV_ROUND_CLOSEST_ULL(div, pc->data->prescaler * NSEC_PER_SEC);

	/*
	 * Lock the period and duty of previous configuration, then
	 * change the duty and period, that would not be effective.
	 */
	ctrl = readl_relaxed(pc->base + pc->data->regs.ctrl);
	if (pc->data->supports_lock) {
		ctrl |= PWM_LOCK_EN;
		writel_relaxed(ctrl, pc->base + pc->data->regs.ctrl);
	}

	writel(period, pc->base + pc->data->regs.period);
	writel(duty, pc->base + pc->data->regs.duty);

	if (pc->data->supports_polarity) {
		ctrl &= ~PWM_POLARITY_MASK;
		if (state->polarity == PWM_POLARITY_INVERSED)
			ctrl |= PWM_DUTY_NEGATIVE | PWM_INACTIVE_POSITIVE;
		else
			ctrl |= PWM_DUTY_POSITIVE | PWM_INACTIVE_NEGATIVE;
	}

	/*
	 * Unlock and set polarity at the same time,
	 * the configuration of duty, period and polarity
	 * would be effective together at next period.
	 */
	if (pc->data->supports_lock)
		ctrl &= ~PWM_LOCK_EN;

	writel(ctrl, pc->base + pc->data->regs.ctrl);
}

static int rockchip_pwm_enable(struct pwm_chip *chip,
			       struct pwm_device *pwm,
			       bool enable)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	u32 enable_conf = pc->data->enable_conf;
	int ret;
	u32 val;

	if (enable) {
		ret = clk_enable(pc->clk);
		if (ret)
			return ret;
	}

	val = readl_relaxed(pc->base + pc->data->regs.ctrl);

	if (enable)
		val |= enable_conf;
	else
		val &= ~enable_conf;

	writel_relaxed(val, pc->base + pc->data->regs.ctrl);

	if (!enable)
		clk_disable(pc->clk);

	return 0;
}

static int rockchip_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			      const struct pwm_state *state)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	struct pwm_state curstate;
	bool enabled;
	int ret = 0;

	ret = clk_enable(pc->pclk);
	if (ret)
		return ret;

	ret = clk_enable(pc->clk);
	if (ret) {
		clk_disable(pc->pclk);
		return ret;
	}

	pwm_get_state(pwm, &curstate);
	enabled = curstate.enabled;

	if (state->polarity != curstate.polarity && enabled &&
	    !pc->data->supports_lock) {
		ret = rockchip_pwm_enable(chip, pwm, false);
		if (ret)
			goto out;
		enabled = false;
	}

	rockchip_pwm_config(chip, pwm, state);
	if (state->enabled != enabled) {
		ret = rockchip_pwm_enable(chip, pwm, state->enabled);
		if (ret)
			goto out;
	}

out:
	clk_disable(pc->clk);
	clk_disable(pc->pclk);

	return ret;
}

static const struct pwm_ops rockchip_pwm_ops = {
	.get_state = rockchip_pwm_get_state,
	.apply = rockchip_pwm_apply,
};

static void rockchip_pwm_v4_write(struct rockchip_pwm_chip *pc,
				  u32 reg, u32 mask, u32 val)
{
	writel((mask << 16) | (val & mask), pc->base + reg);
}

static int rockchip_pwm_v4_ticks(unsigned long rate, const struct pwm_state *state,
				 u32 *period, u32 *duty)
{
	u64 ticks;

	if (!rate || !state->period || state->duty_cycle > state->period)
		return -EINVAL;
	/* Include the rounding addition in the overflow check. */
	if (check_mul_overflow((u64)rate, state->period, &ticks) ||
	    ticks > U64_MAX - NSEC_PER_SEC / 2)
		return -ERANGE;
	ticks = DIV_ROUND_CLOSEST_ULL(ticks, NSEC_PER_SEC);
	if (!ticks || ticks > U32_MAX)
		return -ERANGE;
	*period = ticks;

	/* duty <= period above also bounds this multiplication and rounding. */
	ticks = (u64)rate * state->duty_cycle;
	*duty = DIV_ROUND_CLOSEST_ULL(ticks, NSEC_PER_SEC);
	return 0;
}

static int rockchip_pwm_get_state_v4(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     struct pwm_state *state)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	unsigned long rate;
	u32 val;
	int ret;

	ret = clk_enable(pc->pclk);
	if (ret)
		return ret;
	rate = clk_get_rate(pc->clk);
	if (!rate) {
		ret = -EINVAL;
		goto out;
	}
	/* Only the direct functional clock is used by this standard PWM port. */
	if (readl(pc->base + PWM_V4_CLK_CTRL) & PWM_V4_CLOCK_MASK) {
		ret = -EOPNOTSUPP;
		goto out;
	}
	state->period = DIV_ROUND_CLOSEST_ULL(
		(u64)readl(pc->base + PWM_V4_PERIOD) * NSEC_PER_SEC, rate);
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL(
		(u64)readl(pc->base + PWM_V4_DUTY) * NSEC_PER_SEC, rate);
	val = readl(pc->base + PWM_V4_ENABLE);
	state->enabled = (val & PWM_V4_ENABLE_MASK) == PWM_V4_ENABLE_MASK;
	/* Polarity lives in CTRL, not in the v4 ENABLE register. */
	val = readl(pc->base + PWM_V4_CTRL);
	state->polarity = val & PWM_V4_DUTY_POSITIVE ?
		PWM_POLARITY_NORMAL : PWM_POLARITY_INVERSED;
	ret = 0;
out:
	clk_disable(pc->pclk);
	return ret;
}

/* Caller holds pclk; keep the functional clock until the last period ends. */
static void rockchip_pwm_stop_v4(struct rockchip_pwm_chip *pc)
{
	u64 delay;
	unsigned long rate = clk_get_rate(pc->clk);

	rockchip_pwm_v4_write(pc, PWM_V4_ENABLE, PWM_V4_ENABLE_MASK, 0);
	if (rate) {
		delay = DIV_ROUND_UP_ULL((u64)readl(pc->base + PWM_V4_PERIOD) *
					USEC_PER_SEC, rate);
		while (delay) {
			unsigned long chunk = min_t(u64, delay, USEC_PER_SEC);

			fsleep(chunk);
			delay -= chunk;
		}
	}
	clk_disable(pc->clk);
	pc->v4_enabled = false;
}

static int rockchip_pwm_apply_v4(struct pwm_chip *chip, struct pwm_device *pwm,
				 const struct pwm_state *state)
{
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);
	u32 period, duty, polarity;
	int ret;

	ret = clk_enable(pc->pclk);
	if (ret)
		return ret;
	/* Disabling must work even if the caller supplies no timing. */
	if (!state->enabled) {
		if (pc->v4_enabled)
			rockchip_pwm_stop_v4(pc);
		ret = 0;
		goto out;
	}
	ret = rockchip_pwm_v4_ticks(clk_get_rate(pc->clk), state, &period, &duty);
	if (ret)
		goto out;
	if (!pc->v4_enabled) {
		ret = clk_enable(pc->clk);
		if (ret)
			goto out;
	}

	/* Fixed direct clock, no vendor waveform/oneshot/global mode. */
	rockchip_pwm_v4_write(pc, PWM_V4_ENABLE, PWM_V4_GLOBAL_JOIN, 0);
	rockchip_pwm_v4_write(pc, PWM_V4_CLK_CTRL, PWM_V4_CLOCK_MASK, 0);
	writel(period, pc->base + PWM_V4_PERIOD);
	writel(duty, pc->base + PWM_V4_DUTY);
	polarity = state->polarity == PWM_POLARITY_INVERSED ?
		PWM_V4_INACTIVE_POSITIVE : PWM_V4_DUTY_POSITIVE;
	rockchip_pwm_v4_write(pc, PWM_V4_CTRL,
			      PWM_V4_MODE_MASK | PWM_V4_POLARITY_MASK |
			      PWM_V4_ALIGNED_INVALID,
			      PWM_V4_CONTINUOUS | polarity);
	writel(0, pc->base + PWM_V4_OFFSET);
	writel(0, pc->base + PWM_V4_RPT);
	rockchip_pwm_v4_write(pc, PWM_V4_INT_EN, BIT(2), 0);
	rockchip_pwm_v4_write(pc, PWM_V4_ENABLE, PWM_V4_UPDATE, PWM_V4_UPDATE);
	if (!pc->v4_enabled) {
		rockchip_pwm_v4_write(pc, PWM_V4_ENABLE,
				      PWM_V4_ENABLE_MASK, PWM_V4_ENABLE_MASK);
		pc->v4_enabled = true;
	}
	ret = 0;
out:
	clk_disable(pc->pclk);
	return ret;
}

static const struct pwm_ops rockchip_pwm_ops_v4 = {
	.get_state = rockchip_pwm_get_state_v4,
	.apply = rockchip_pwm_apply_v4,
};

static const struct rockchip_pwm_data pwm_data_v1 = {
	.regs = {
		.duty = 0x04,
		.period = 0x08,
		.cntr = 0x00,
		.ctrl = 0x0c,
	},
	.prescaler = 2,
	.supports_polarity = false,
	.supports_lock = false,
	.enable_conf = PWM_CTRL_OUTPUT_EN | PWM_CTRL_TIMER_EN,
};

static const struct rockchip_pwm_data pwm_data_v2 = {
	.regs = {
		.duty = 0x08,
		.period = 0x04,
		.cntr = 0x00,
		.ctrl = 0x0c,
	},
	.prescaler = 1,
	.supports_polarity = true,
	.supports_lock = false,
	.enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE |
		       PWM_CONTINUOUS,
};

static const struct rockchip_pwm_data pwm_data_vop = {
	.regs = {
		.duty = 0x08,
		.period = 0x04,
		.cntr = 0x0c,
		.ctrl = 0x00,
	},
	.prescaler = 1,
	.supports_polarity = true,
	.supports_lock = false,
	.enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE |
		       PWM_CONTINUOUS,
};

static const struct rockchip_pwm_data pwm_data_v3 = {
	.regs = {
		.duty = 0x08,
		.period = 0x04,
		.cntr = 0x00,
		.ctrl = 0x0c,
	},
	.prescaler = 1,
	.supports_polarity = true,
	.supports_lock = true,
	.enable_conf = PWM_OUTPUT_LEFT | PWM_LP_DISABLE | PWM_ENABLE |
		       PWM_CONTINUOUS,
};

static const struct rockchip_pwm_data pwm_data_v4 = {
	.regs = {
		.duty = PWM_V4_DUTY,
		.period = PWM_V4_PERIOD,
		.ctrl = PWM_V4_CTRL,
	},
	.prescaler = 1,
	.supports_polarity = true,
	.supports_lock = true,
	.v4 = true,
	.enable_conf = PWM_V4_ENABLE_MASK,
};

static const struct of_device_id rockchip_pwm_dt_ids[] = {
	{ .compatible = "rockchip,rk2928-pwm", .data = &pwm_data_v1},
	{ .compatible = "rockchip,rk3288-pwm", .data = &pwm_data_v2},
	{ .compatible = "rockchip,vop-pwm", .data = &pwm_data_vop},
	{ .compatible = "rockchip,rk3328-pwm", .data = &pwm_data_v3},
	{ .compatible = "rockchip,rk3576-pwm", .data = &pwm_data_v4},
	{ .compatible = "rockchip,rv1103b-pwm", .data = &pwm_data_v4},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rockchip_pwm_dt_ids);

static int rockchip_pwm_probe(struct platform_device *pdev)
{
	struct pwm_chip *chip;
	struct rockchip_pwm_chip *pc;
	u32 enable_conf, ctrl;
	bool enabled;
	int ret, count;

	chip = devm_pwmchip_alloc(&pdev->dev, 1, sizeof(*pc));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	pc = to_rockchip_pwm_chip(chip);

	pc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pc->base))
		return PTR_ERR(pc->base);

	pc->clk = devm_clk_get(&pdev->dev, "pwm");
	if (IS_ERR(pc->clk)) {
		pc->clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(pc->clk))
			return dev_err_probe(&pdev->dev, PTR_ERR(pc->clk),
					     "Can't get PWM clk\n");
	}

	count = of_count_phandle_with_args(pdev->dev.of_node,
					   "clocks", "#clock-cells");
	if (count >= 2)
		pc->pclk = devm_clk_get(&pdev->dev, "pclk");
	else
		pc->pclk = pc->clk;

	if (IS_ERR(pc->pclk))
		return dev_err_probe(&pdev->dev, PTR_ERR(pc->pclk), "Can't get APB clk\n");

	ret = clk_prepare_enable(pc->clk);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Can't prepare enable PWM clk\n");

	ret = clk_prepare_enable(pc->pclk);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "Can't prepare enable APB clk\n");
		goto err_clk;
	}

	platform_set_drvdata(pdev, chip);

	pc->data = device_get_match_data(&pdev->dev);
	chip->ops = pc->data->v4 ? &rockchip_pwm_ops_v4 : &rockchip_pwm_ops;

	enable_conf = pc->data->enable_conf;
	ctrl = readl_relaxed(pc->base + (pc->data->v4 ? PWM_V4_ENABLE :
						    pc->data->regs.ctrl));
	enabled = (ctrl & enable_conf) == enable_conf;
	if (pc->data->v4) {
		struct pinctrl *pinctrl;

		if ((readl(pc->base + PWM_V4_VERSION) >> 24) != 4) {
			ret = dev_err_probe(&pdev->dev, -ENODEV, "Unsupported PWM version\n");
			goto err_pclk;
		}
		if (enabled &&
		    ((ctrl & PWM_V4_GLOBAL_JOIN) ||
		     (readl(pc->base + PWM_V4_CLK_CTRL) & PWM_V4_CLOCK_MASK) ||
		     (readl(pc->base + PWM_V4_CTRL) & PWM_V4_MODE_MASK) != PWM_V4_CONTINUOUS)) {
			ret = dev_err_probe(&pdev->dev, -EOPNOTSUPP,
					     "Unsupported active PWM clock/mode\n");
			goto err_pclk;
		}
		if (!clk_get_rate(pc->clk)) {
			ret = dev_err_probe(&pdev->dev, -EINVAL, "Invalid PWM clock rate\n");
			goto err_pclk;
		}
		if (!enabled)
			rockchip_pwm_v4_write(pc, PWM_V4_CLK_CTRL, PWM_V4_CLOCK_MASK, 0);
		/* Accept the working RV DT's active state as well as default. */
		if (of_property_match_string(pdev->dev.of_node, "pinctrl-names", "active") >= 0) {
			pinctrl = devm_pinctrl_get_select(&pdev->dev, "active");
			if (IS_ERR(pinctrl)) {
				ret = dev_err_probe(&pdev->dev, PTR_ERR(pinctrl),
						     "Can't select PWM pins\n");
				goto err_pclk;
			}
		}
		pc->v4_enabled = enabled;
	}

	ret = pwmchip_add(chip);
	if (ret < 0) {
		dev_err_probe(&pdev->dev, ret, "pwmchip_add() failed\n");
		goto err_pclk;
	}

	/* Keep the PWM clk enabled if the PWM appears to be up and running. */
	if (!enabled)
		clk_disable(pc->clk);

	clk_disable(pc->pclk);

	return 0;

err_pclk:
	clk_disable_unprepare(pc->pclk);
err_clk:
	clk_disable_unprepare(pc->clk);

	return ret;
}

static void rockchip_pwm_remove(struct platform_device *pdev)
{
	struct pwm_chip *chip = platform_get_drvdata(pdev);
	struct rockchip_pwm_chip *pc = to_rockchip_pwm_chip(chip);

	pwmchip_remove(chip);

	if (pc->data->v4 && pc->v4_enabled) {
		if (!clk_enable(pc->pclk)) {
			rockchip_pwm_stop_v4(pc);
			clk_disable(pc->pclk);
		} else {
			dev_warn(&pdev->dev, "Can't stop PWM: APB clock failed\n");
			clk_disable(pc->clk);
		}
	}
	clk_unprepare(pc->pclk);
	clk_unprepare(pc->clk);
}

static struct platform_driver rockchip_pwm_driver = {
	.driver = {
		.name = "rockchip-pwm",
		.of_match_table = rockchip_pwm_dt_ids,
	},
	.probe = rockchip_pwm_probe,
	.remove = rockchip_pwm_remove,
};
module_platform_driver(rockchip_pwm_driver);

MODULE_AUTHOR("Beniamino Galvani <b.galvani@gmail.com>");
MODULE_DESCRIPTION("Rockchip SoC PWM driver");
MODULE_LICENSE("GPL v2");
