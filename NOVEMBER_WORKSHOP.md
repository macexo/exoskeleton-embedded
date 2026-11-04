# November SPI workshop: member guide

**Discord → GitHub issue → this guide.** This is the only how-to guide you need. Deadlines are in the issue. Target: **NUCLEO-F446RE + LSM6DSO32**.

```text
Copy starter → write two functions → draft PR → workshop testing → update PR
```

## 1. Set up and copy

Install [VS Code](https://code.visualstudio.com/download), Microsoft's [C/C++ extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools), and [Git](https://git-scm.com/install/). Check `git --version` and GitHub sign-in. **Use VS Code to read and edit this week.** At the workshop, build the whole STM32 project in CubeIDE using its ARM compiler, then run it on the board. Installing CubeIDE beforehand is optional; no Python or separate laptop C compiler is required.

**Example — replace `majock` with your own unique lowercase name in the commands and folders below.** This is an individual assignment: one folder and PR per member.

```sh
git clone https://github.com/macexo/exoskeleton-embedded.git
cd exoskeleton-embedded
git switch -c feat/spi-imu-majock
```

Open the repo in VS Code. Copy [submissions/_template/](workshops/spi-imu/submissions/_template/) next to itself and rename it using your own name. **Example folder layout, using `majock`:**

```text
workshops/spi-imu/submissions/majock/
├── imu_spi.c   ← your implementation
└── README.md   ← your notes; replace YOUR_NAME
```

**Only edit these two files before the workshop.** The header is finished and read-only. The larger `firmware/` folder supplies the board initialization, ST libraries, and CubeIDE build configuration; the leads will explain it in person.

## 2. Write two functions

Follow the numbered TODOs in your C file:

| Order | Function | Task |
| --- | --- | --- |
| 1 | `imu_check_identity()` | Call the supplied read and verify the identity |
| 2 | `imu_write_register()` | Adapt the read example to send an address/value |

Keep the supplied read and [shared header](workshops/spi-imu/firmware/Core/Inc/imu_spi.h) unchanged. The header defines types and expected behavior. C hints: `uint8_t` holds a byte; `&identity` lets a function fill that variable. Check returned status before using received data.

Use the [ST datasheet](https://www.st.com/resource/en/datasheet/lsm6dso32.pdf): **5.1.2** (SPI), **9.11** (identity), and **9.14** (write/readback register).

| Register address | Name | Use |
| --- | --- | --- |
| `0x0F` | WHO_AM_I | Read-only; expect `0x6C` |
| `0x12` | CTRL3_C | Workshop write/readback: `0x44` |

Registers are inside the IMU; GPIO pins connect boards. The workshop project will supply SPI/CS through `imu_device_t`. Before the workshop, edit only your submission folder; the complete board project is supplied in `firmware/`.

## 3. Submit a draft PR

In your README, record progress/questions and briefly explain bit 7, the dummy byte, `rx[1]`, and CS in the worked read. Mark build/hardware results untested. No new research report or pre-workshop testing is required; missing HAL/include-path diagnostics can wait for CubeIDE setup.

**Example — replace every `majock` below with the name used for your own folder and branch:**

```sh
git add workshops/spi-imu/submissions/majock/
git commit -m "feat(workshop): draft majock SPI IMU driver"
git push -u origin feat/spi-imu-majock
```

Complete Git name/email or authentication prompts; flag access problems to the leads. On GitHub: **New pull request → base `main` / compare your branch → Create draft pull request**. Include `Related to #<assignment issue number>`, your README link, and remaining TODOs. If blocked, submit your progress before the deadline.

## 4. At the workshop

With the leads, import `workshops/spi-imu/firmware/` using **File → Import → General → Existing Projects into Workspace**, leaving **Copy projects into workspace** unchecked. The project appears as **imu-spi**; use a fresh workspace if that name is already imported from elsewhere.

Paste your code into its `Core/Src/imu_spi.c`. The read-only header and the demo hook are already included. The leads will explain the surrounding initialization and Git workflow.

```text
Build (hammer) → flash/resume with leads → check identity and write/readback
```

Use the `workshop identity`, `workshop write`, and `workshop readback` UART lines to check your functions. The untouched starter reports TODO; the supplied motion stream alone does not establish completion.

**Copy fixes back into your submission.** Commit the code; record its commit ID, wiring, UART evidence, and limitations in your README. Commit notes and push to the same PR. The leads review and merge each member's folder independently.

<details>
<summary>Optional workshop wiring/reference</summary>

Confirm this planned wiring with the leads before powering the breakout. These are STM32 GPIO names.

| STM32 GPIO | IMU signal |
| --- | --- |
| PC10 | SCL/SPC — SPI3 clock |
| PB4 | SDO — SPI3 MISO |
| PB5 | SDA/SDI — SPI3 MOSI |
| PB6 | CS |
| GND | GND |
| 3.3 V | Breakout power input confirmed by leads |

Reference settings: SPI3 mode 3, 8-bit, MSB-first, software CS; 45 MHz APB1 ÷ 16 = **2.8125 MHz**. UART2 uses ST-LINK virtual COM at **115200, 8N1**. Follow this reference project's wiring rather than last year's custom PCB, which is I²C-only for this workshop.

</details>

Configuration and sample reading may follow once everyone is comfortable; they are outside this week's assignment.
