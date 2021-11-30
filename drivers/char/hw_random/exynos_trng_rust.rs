// SPDX-License-Identifier: GPL-2.0

//! RNG driver for Exynos TRNGs
//!
//! Author: Maciej Falkowski <m.falkowski@samsung.com>
//!
//! Copyright 2022 (c) Samsung Electronics Software, Inc.
//!
//! Based on the Exynos TRNG driver drivers/char/hw_random/exynos-rng by
//! Łukasz Stelmach <l.stelmach@samsung.com>

use core::{cmp::min, convert::TryInto, str};

use kernel::{
    c_str,
    clk::EnabledClk,
    declare_hwrng_operations, device,
    device::RawDevice,
    hwrng,
    io_mem::IoMem,
    module_platform_driver, of, platform, pm_runtime, power,
    prelude::*,
    sync::{Ref, RefBorrow, SpinLock},
};

const EXYNOS_TRNG_CLKDIV: usize = 0x0;

const EXYNOS_TRNG_CTRL: usize = 0x20;
const EXYNOS_TRNG_CTRL_RNGEN: usize = 0x1 << 31; // BIT(31)

const EXYNOS_REG_SIZE: usize = 0x100;
const EXYNOS_TRNG_POST_CTRL: usize = 0x30;
const _EXYNOS_TRNG_ONLINE_CTRL: usize = 0x40;
const _EXYNOS_TRNG_ONLINE_STAT: usize = 0x44;
const _EXYNOS_TRNG_ONLINE_MAXCHI2: usize = 0x48;
const EXYNOS_TRNG_FIFO_CTRL: usize = 0x50;
const EXYNOS_TRNG_FIFO_0: usize = 0x80;
const _EXYNOS_TRNG_FIFO_1: usize = 0x84;
const _EXYNOS_TRNG_FIFO_2: usize = 0x88;
const _EXYNOS_TRNG_FIFO_3: usize = 0x8c;
const _EXYNOS_TRNG_FIFO_4: usize = 0x90;
const _EXYNOS_TRNG_FIFO_5: usize = 0x94;
const _EXYNOS_TRNG_FIFO_6: usize = 0x98;
const _EXYNOS_TRNG_FIFO_7: usize = 0x9c;
const EXYNOS_TRNG_FIFO_LEN: u32 = 8;
const EXYNOS_TRNG_CLOCK_RATE: usize = 500000;

struct ExynosTrngDevice;

struct ExynosTrngDataInner {
    clk: EnabledClk,
}

struct ExynosTrngData {
    dev: device::Device,
    disable: pm_runtime::DisableGuard,
    inner: SpinLock<ExynosTrngDataInner>,
}

struct ExynosTrngResources {
    mem: IoMem<EXYNOS_REG_SIZE>,
}

type DeviceData =
    device::Data<hwrng::Registration<ExynosTrngDevice>, ExynosTrngResources, ExynosTrngData>;

impl hwrng::Operations for ExynosTrngDevice {
    declare_hwrng_operations!(init);

    type Data = Ref<DeviceData>;

    fn init(trng: RefBorrow<'_, DeviceData>) -> Result {
        let sss_rate = trng.inner.lock().clk.get_rate();

        // For most TRNG circuits the clock frequency of under 500 kHz
        // is safe.
        let mut val = sss_rate / (EXYNOS_TRNG_CLOCK_RATE * 2);
        if val > 0x7fff {
            dev_err!(trng.dev, "clock divider too large: {}\n", val);
            return Err(Error::ERANGE);
        }

        let trng = trng.resources().ok_or(Error::ENXIO)?;

        val <<= 1;
        trng.mem.writel_relaxed(val.try_into()?, EXYNOS_TRNG_CLKDIV);

        // Enable the generator.
        val = EXYNOS_TRNG_CTRL_RNGEN;
        trng.mem.writel_relaxed(val.try_into()?, EXYNOS_TRNG_CTRL);

        // Disable post-processing. /dev/hwrng is supposed to deliver
        // unprocessed data.
        trng.mem.writel_relaxed(0, EXYNOS_TRNG_POST_CTRL);

        Ok(())
    }

    fn read(trng: RefBorrow<'_, DeviceData>, data: &mut [u8], _wait: bool) -> Result<u32> {
        let trng = trng.resources().ok_or(Error::ENXIO)?;
        let max: u32 = min(data.len().try_into()?, EXYNOS_TRNG_FIFO_LEN * 4);

        trng.mem.writel_relaxed(max * 8, EXYNOS_TRNG_FIFO_CTRL);

        let _ =
            trng.mem
                .readl_poll_timeout(EXYNOS_TRNG_FIFO_CTRL, |val| *val == 0, 200, 1000000)?;

        trng.mem
            .try_memcpy_fromio(&mut data[..max as _], EXYNOS_TRNG_FIFO_0)?;

        Ok(max)
    }
}

struct ExynosTrngDriver;
impl platform::Driver for ExynosTrngDriver {
    type Data = Ref<DeviceData>;

    kernel::define_of_id_table! {(), [
        (of::DeviceId::Compatible(b"samsung,exynos5250-trng"), None),
    ]}

    fn probe(pdev: &mut platform::Device, _id_info: Option<&Self::IdInfo>) -> Result<Self::Data> {
        let clk = pdev.clk_get(Some(c_str!("secss")))?;
        let clk = clk.prepare_enable()?;

        // SAFETY: Dma operations are not used.
        let mem: IoMem<EXYNOS_REG_SIZE> = unsafe { pdev.ioremap_resource(0) }?;

        let disable = pm_runtime::enable(pdev);
        pm_runtime::resume_and_get(pdev)?;

        let mut data = kernel::new_device_data!(
            hwrng::Registration::new(),
            ExynosTrngResources { mem },
            ExynosTrngData {
                dev: device::Device::from_dev(pdev),
                disable,
                // SAFETY: SpinLock is initialized in the same context later.
                inner: unsafe { SpinLock::new(ExynosTrngDataInner { clk }) },
            },
            "ExynosTrng::Registrations"
        )?;

        let data_inner = unsafe { data.as_mut().map_unchecked_mut(|d| &mut (**d).inner) };
        kernel::spinlock_init!(data_inner, "ExynosTrng::inner");

        let data = Ref::<DeviceData>::from(data);

        data.registrations()
            .ok_or(Error::ENXIO)?
            .as_pinned_mut()
            .register(fmt!("{}", pdev.name()), 0, data.clone())?;

        Ok(data)
    }

    fn remove(trng: &Self::Data) -> Result {
        pm_runtime::put_sync(&trng.dev);
        drop(&trng.disable);
        Ok(())
    }
}

impl power::Operations for ExynosTrngDevice {
    type Data = Ref<DeviceData>;

    fn suspend(trng: RefBorrow<'_, DeviceData>) -> Result {
        pm_runtime::put_sync(&trng.dev);
        Ok(())
    }

    fn resume(trng: RefBorrow<'_, DeviceData>) -> Result {
        pm_runtime::resume_and_get(&trng.dev)?;
        Ok(())
    }
}

module_platform_driver! {
    type: ExynosTrngDriver,
    name: b"exynos_trng_rust",
    author: b"Maciej Falkowski <m.falkowski@samsung.com>",
    description: b"H/W TRNG driver for Exynos chips",
    license: b"GPL v2",
}
