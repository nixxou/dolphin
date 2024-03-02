// Copyright 2010 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/WiimoteEmu/WiimoteEmu.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string_view>

#include <fmt/format.h>

#include "Common/Assert.h"
#include "Common/Common.h"
#include "Common/CommonTypes.h"
#include "Common/Config/Config.h"
#include "Common/FileUtil.h"
#include "Common/Logging/Log.h"
#include "Common/MathUtil.h"
#include "Common/MsgHandler.h"

#include "Core/Config/MainSettings.h"
#include "Core/Core.h"
#include "Core/HW/Wiimote.h"
#include "Core/Movie.h"
#include "Core/System.h"

#include "Core/HW/WiimoteCommon/WiimoteConstants.h"
#include "Core/HW/WiimoteCommon/WiimoteHid.h"
#include "Core/HW/WiimoteEmu/DesiredWiimoteState.h"
#include "Core/HW/WiimoteEmu/Extension/Classic.h"
#include "Core/HW/WiimoteEmu/Extension/DesiredExtensionState.h"
#include "Core/HW/WiimoteEmu/Extension/DrawsomeTablet.h"
#include "Core/HW/WiimoteEmu/Extension/Drums.h"
#include "Core/HW/WiimoteEmu/Extension/Guitar.h"
#include "Core/HW/WiimoteEmu/Extension/Nunchuk.h"
#include "Core/HW/WiimoteEmu/Extension/Shinkansen.h"
#include "Core/HW/WiimoteEmu/Extension/TaTaCon.h"
#include "Core/HW/WiimoteEmu/Extension/Turntable.h"
#include "Core/HW/WiimoteEmu/Extension/UDrawTablet.h"

#include "InputCommon/ControllerEmu/Control/Input.h"
#include "InputCommon/ControllerEmu/Control/Output.h"
#include "InputCommon/ControllerEmu/ControlGroup/Attachments.h"
#include "InputCommon/ControllerEmu/ControlGroup/Buttons.h"
#include "InputCommon/ControllerEmu/ControlGroup/ControlGroup.h"
#include "InputCommon/ControllerEmu/ControlGroup/Cursor.h"
#include "InputCommon/ControllerEmu/ControlGroup/Force.h"
#include "InputCommon/ControllerEmu/ControlGroup/IMUAccelerometer.h"
#include "InputCommon/ControllerEmu/ControlGroup/IMUCursor.h"
#include "InputCommon/ControllerEmu/ControlGroup/IMUGyroscope.h"
#include "InputCommon/ControllerEmu/ControlGroup/ModifySettingsButton.h"
#include "InputCommon/ControllerEmu/ControlGroup/Tilt.h"
#include "Core/ConfigManager.h"
#include "Core/PowerPC/MMU.h"
#include "Core/Config/SYSCONFSettings.h"
#include "Core/AchievementManager.h"
#include <Core/MameHookerProxy.h>

namespace WiimoteEmu
{
using namespace WiimoteCommon;

static const u16 button_bitmasks[] = {
    Wiimote::BUTTON_A,     Wiimote::BUTTON_B,    Wiimote::BUTTON_ONE, Wiimote::BUTTON_TWO,
    Wiimote::BUTTON_MINUS, Wiimote::BUTTON_PLUS, Wiimote::BUTTON_HOME};

static const u16 dpad_bitmasks[] = {Wiimote::PAD_UP, Wiimote::PAD_DOWN, Wiimote::PAD_LEFT,
                                    Wiimote::PAD_RIGHT};

static const u16 dpad_sideways_bitmasks[] = {Wiimote::PAD_RIGHT, Wiimote::PAD_LEFT, Wiimote::PAD_UP,
                                             Wiimote::PAD_DOWN};

void Wiimote::Reset()
{
  const bool want_determinism = Core::WantsDeterminism();

  SetRumble(false);

  // Wiimote starts in non-continuous CORE mode:
  m_reporting_mode = InputReportID::ReportCore;
  m_reporting_continuous = false;

  m_speaker_mute = false;

  // EEPROM

  // TODO: This feels sketchy, this needs to properly handle the case where the load and the write
  // happen under different Wii Roots and/or determinism modes.

  std::string eeprom_file = (File::GetUserPath(D_SESSION_WIIROOT_IDX) + "/" + GetName() + ".bin");
  if (!want_determinism && m_eeprom_dirty)
  {
    // Write out existing EEPROM
    INFO_LOG_FMT(WIIMOTE, "Wrote EEPROM for {}", GetName());
    std::ofstream file;
    File::OpenFStream(file, eeprom_file, std::ios::binary | std::ios::out);
    file.write(reinterpret_cast<char*>(m_eeprom.data.data()), EEPROM_FREE_SIZE);
    file.close();

    m_eeprom_dirty = false;
  }
  m_eeprom = {};

  if (!want_determinism && File::Exists(eeprom_file))
  {
    // Read existing EEPROM
    std::ifstream file;
    File::OpenFStream(file, eeprom_file, std::ios::binary | std::ios::in);
    file.read(reinterpret_cast<char*>(m_eeprom.data.data()), EEPROM_FREE_SIZE);
    file.close();
  }
  else
  {
    // Load some default data.

    // IR calibration:
    std::array<u8, 11> ir_calibration = {
        // Point 1
        IR_LOW_X & 0xFF,
        IR_LOW_Y & 0xFF,
        // Mix
        ((IR_LOW_Y & 0x300) >> 2) | ((IR_LOW_X & 0x300) >> 4) | ((IR_LOW_Y & 0x300) >> 6) |
            ((IR_HIGH_X & 0x300) >> 8),
        // Point 2
        IR_HIGH_X & 0xFF,
        IR_LOW_Y & 0xFF,
        // Point 3
        IR_HIGH_X & 0xFF,
        IR_HIGH_Y & 0xFF,
        // Mix
        ((IR_HIGH_Y & 0x300) >> 2) | ((IR_HIGH_X & 0x300) >> 4) | ((IR_HIGH_Y & 0x300) >> 6) |
            ((IR_LOW_X & 0x300) >> 8),
        // Point 4
        IR_LOW_X & 0xFF,
        IR_HIGH_Y & 0xFF,
        // Checksum
        0x00,
    };
    UpdateCalibrationDataChecksum(ir_calibration, 1);
    m_eeprom.ir_calibration_1 = ir_calibration;
    m_eeprom.ir_calibration_2 = ir_calibration;

    // Accel calibration:
    // Last byte is a checksum.
    std::array<u8, 10> accel_calibration = {
        ACCEL_ZERO_G, ACCEL_ZERO_G, ACCEL_ZERO_G, 0, ACCEL_ONE_G, ACCEL_ONE_G, ACCEL_ONE_G, 0, 0, 0,
    };
    UpdateCalibrationDataChecksum(accel_calibration, 1);
    m_eeprom.accel_calibration_1 = accel_calibration;
    m_eeprom.accel_calibration_2 = accel_calibration;

    // TODO: Is this needed?
    // Data of unknown purpose:
    constexpr std::array<u8, 24> EEPROM_DATA_16D0 = {
        0x00, 0x00, 0x00, 0xFF, 0x11, 0xEE, 0x00, 0x00, 0x33, 0xCC, 0x44, 0xBB,
        0x00, 0x00, 0x66, 0x99, 0x77, 0x88, 0x00, 0x00, 0x2B, 0x01, 0xE8, 0x13};
    m_eeprom.unk_2 = EEPROM_DATA_16D0;

    std::string mii_file = File::GetUserPath(D_SESSION_WIIROOT_IDX) + "/mii.bin";
    if (File::Exists(mii_file))
    {
      // Import from the existing mii.bin file, if present
      std::ifstream file;
      File::OpenFStream(file, mii_file, std::ios::binary | std::ios::in);
      file.read(reinterpret_cast<char*>(m_eeprom.mii_data_1.data()), m_eeprom.mii_data_1.size());
      m_eeprom.mii_data_2 = m_eeprom.mii_data_1;
      file.close();
    }
  }

  m_read_request = {};

  // Initialize i2c bus:
  m_i2c_bus.Reset();
  m_i2c_bus.AddSlave(&m_speaker_logic);
  m_i2c_bus.AddSlave(&m_camera_logic);

  // Reset extension connections to NONE:
  m_is_motion_plus_attached = false;
  m_active_extension = ExtensionNumber::NONE;
  m_extension_port.AttachExtension(GetNoneExtension());
  m_motion_plus.GetExtPort().AttachExtension(GetNoneExtension());

  if (!want_determinism)
  {
    // Switch to desired M+ status and extension (if any).
    // M+ and EXT are reset on attachment.
    HandleExtensionSwap(static_cast<ExtensionNumber>(m_attachments->GetSelectedAttachment()),
                        m_motion_plus_setting.GetValue());
  }

  // Reset sub-devices.
  m_speaker_logic.Reset();
  m_camera_logic.Reset();

  m_status = {};

  // A real wii remote does not normally send a status report on connection.
  // But if an extension is already attached it does send one.
  // Clearing this initially will simulate that on the first update cycle.
  m_status.extension = 0;

  // Dynamics:
  m_swing_state = {};
  m_tilt_state = {};
  m_point_state = {};
  m_shake_state = {};

  m_imu_cursor_state = {};
}

void Wiimote::threadOutputs()
{
  const long max_time_lastPress = 100000;
  DEBUG_LOG_FMT(ACHIEVEMENTS, "THREAD {} : Thread active", m_index);
  while (true)
  {
    if (quitThread)
      break;
    std::string title = SConfig::GetInstance().GetGameID();

    if (lastActiveGame != title)
    {
      if (serialPort != INVALID_HANDLE_VALUE)
      {
        Wiimote::SendComMessage("E");
        CloseHandle(serialPort);
        serialPort = INVALID_HANDLE_VALUE;
      }

      lastActiveGame = title;
      lastRatio = 0;
      triggerIsActive = false;
      triggerLastPress = 0;
      triggerLastRelease = 0;
      lastAmmo = INT32_MAX;
      lastWeapon = 0;
      lastCharged = 0;
      lastOther1 = 0;
      lastOther2 = 0;
      fullAutoActive = false;
      activeRecoil = false;
      lastRatio = 0;
      if (title != "" && title != "00000000" && Config::Get(Config::SYSCONF_WIDESCREEN)) lastRatio = 1;

      if (title != "" && title != "00000000"){
        gun4irComPort = static_cast<int>(std::floor(m_ir->m_gun4ircom_setting.GetValue()));
        bool validcom = false;
        if (gun4irComPort > 0)
        {
          validcom = true;
          std::string serialPortName = "COM" + std::to_string(gun4irComPort);
          if (gun4irComPort >= 10)
          {
            serialPortName = "\\\\.\\COM" + std::to_string(gun4irComPort);
          }
          serialPort = CreateFileA(serialPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL,
                                     OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

          if (serialPort == INVALID_HANDLE_VALUE)
          {
            validcom = false;
          }
          if (validcom)
          {
            DCB dcbSerialParams = {0};
            dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

            if (!GetCommState(serialPort, &dcbSerialParams))
            {
              validcom = false;
            }
            if (validcom)
            {
              dcbSerialParams.BaudRate = 9600;
              dcbSerialParams.ByteSize = 8;
              dcbSerialParams.StopBits = ONESTOPBIT;
              dcbSerialParams.Parity = NOPARITY;
            }
            if (!SetCommState(serialPort, &dcbSerialParams))
            {
              validcom = false;
            }
          }
        }
        if (validcom)
        {
          Wiimote::SendComMessage("S6");
        }
        else
        {
          serialPort = INVALID_HANDLE_VALUE;
        }
}
    }

    if (!activeRecoil && triggerLastPress > 0)
      activeRecoil = true;

    if (!activeRecoil)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    bool valid_query = false;
    std::string output_signal = "";

    std::chrono::microseconds::rep timestamp =
    std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();

    // Not Compatible list :
    // Dino Strike
    // Cocoto Magic Circus
    // Gunblade NY & L.A. Machineguns Arcade Hits Pack (USA) (En,Fr,Es)
    // Martian Panic
    // Nerf N Strike elite
    // Pirate Blast
    // Reload
    // Remington Super Slam Hunting - Alaska (USA)
    // Remington Super Slam Hunting - North America (USA)
    // Sin & Punishment - Star Successor (USA)
    // Sniper Elite

    //Attack of the Movies 3-D (USA)
    if (title == "S3AE5G")
    {
      int ammoCount = 0;
      int weaponType = 0;
      int max_player = 2;
      /* std::chrono::microseconds::rep timestamp =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();
      */
      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_weaponType = PowerPC::MMU::HostTryReadU16(guard, 0x80eaa99e);
          if (!blr_weaponType)
            valid_query = false;
          else
            weaponType = blr_weaponType->value;

          auto blr_ammo = PowerPC::MMU::HostTryReadU16(guard, 0x80eaa8ca + (weaponType * 0x3c));
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_weaponType = PowerPC::MMU::HostTryReadU16(guard, 0x809F0FFE);
          if (!blr_weaponType)
            valid_query = false;
          else
            weaponType = blr_weaponType->value;

          auto blr_ammo = PowerPC::MMU::HostTryReadU16(guard, 0x809F0F2A + (weaponType * 0x3c));
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }

      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }

      //Old Style
     /*
      if (valid_query)
      {
        if (fullAutoActive && (!triggerIsActive || ammoCount == 0))
        {
          output_signal = "machinegun_off";
          fullAutoActive = false;
        }
        else
        {
          if (ammoCount < lastAmmo)
          {
            long long diff = timestamp - triggerLastPress;
            DEBUG_LOG_FMT(ACHIEVEMENTS, "diff {}", diff);
            if (diff < max_time_lastPress)
            {
              if (weaponType >1)
              {
                if (!fullAutoActive)
                {
                  fullAutoActive = true;
                  output_signal = "machinegun_on";
                }
              }
              else
              {
                if (weaponType == 1)
                  output_signal = "tripleshot";
                else
                  output_signal = "gunshot";
              }

              triggerLastPress = 0;
            }
          }
        }
        lastAmmo = ammoCount;
      }
      */
    }

    //Chicken Shoot (USA)
    if (title == "RCSE20") 
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU16(guard, 0x8017fa8a);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU16(guard, 0x8017faa2);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }

    //Conduit 2 (USA)
    //Note : Support for 1st player only  
    if (title == "SC2E8P")
    {
      int ammoCount = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x8087EEA0);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress - 0x15ED));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }

    //Dead Space - Extraction (USA)
    if (title == "RZJE69")
    {
      int ammoCount = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x804B8BF3);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }
    
    //Deer Drive Legends (USA)
    if (title == "SUNEYG")  
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadF32(guard, 0x903d53ac);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU16(guard, 0x903d576c);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }

    //Eco Shooter - Plant 530 (USA) (WiiWare)
    if (title == "W6BE01")
    {
      int ammoCount = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8028415B);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }

    //Fast Draw Showdown (USA) (WiiWare)
    if (title == "WFAEJS")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x801C7A8B);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x801C7A8F);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }

    //Ghost Squad (USA)
    if (title == "RGSE8P")
    {
      int ammoCount = 0;
      int max_player = 4;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x80507410);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x4b));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
        }
        if (m_index == 1)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x80507410);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x69b));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
        }
        if (m_index == 2)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x80507410);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0xceb));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
        }
        if (m_index == 3)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x80507410);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x133B));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
        }
      }

      if (valid_query)
      {
        if (ammoCount < lastAmmo)
        {
          if (triggerIsActive)
          {
            output_signal = "gunshot";
          }
        }
        lastAmmo = ammoCount;
      }
    }

    //Gunslingers (USA) (Rev 1)
    if (title == "SW7EVN")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x80A6A853);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x80A6A7D3);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo)
        {
          if (triggerIsActive)
          {
            output_signal = "gunshot";
          }
        }
        lastAmmo = ammoCount;
      }
    }

    //Heavy Fire - Black Arms (USA) (WiiWare)
    //Note : Don't support the gun at the start of the game with unlimited ammo
    if (title == "WHYETY")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8054F7A7);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8054F7E3);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }

        if (valid_query)
        {
          if (ammoCount < lastAmmo)
          {
            if (triggerIsActive)
            {
              output_signal = "gunshot";
              triggerLastPress = 0;
            }
          }
          lastAmmo = ammoCount;
        }
      }
    }

    // Heavy Fire - Special Operations (USA) (WiiWare)
    //  Note : Don't support the gun at the start of the game with unlimited ammo
    if (title == "WHFETY")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8045AFC7);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8045B003);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }

        if (valid_query)
        {
          if (ammoCount < lastAmmo)
          {
            if (triggerIsActive)
            {
              output_signal = "gunshot";
              triggerLastPress = 0;
            }
          }
          lastAmmo = ammoCount;
        }
      }
    }

    //Heavy Fire - Afghanistan (USA)
    if (title == "SH4EFP")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8055BD43);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8055C00B);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }

        if (valid_query)
        {
          if (ammoCount < lastAmmo)
          {
            if (triggerIsActive)
            {
              output_signal = "gunshot";
              triggerLastPress = 0;
            }
          }
          lastAmmo = ammoCount;
        }
      }
    }

    //Jurassic - The Hunted (USA)
    if (title == "R8XE52")
    {
      int ammoCount = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x807798E0);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x267));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
        }
      }

      if (valid_query)
      {
        if (ammoCount < lastAmmo)
        {
          if (triggerIsActive)
          {
            output_signal = "gunshot";
          }
        }
        lastAmmo = ammoCount;
      }
    }

    //Link's Crossbow Training (USA) (Rev 1)
    //Note : Does not detect when it go to fullauto (buff)
    if (title == "RZPE01")
    {
      int gunStatus = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_gunStatus = PowerPC::MMU::HostTryReadU8(guard, 0x8036040D);
          if (!blr_gunStatus)
            valid_query = false;
          else
            gunStatus = blr_gunStatus->value;
        }
      }

      if (valid_query && gunStatus > 1)
      {
        long long diffrlz = timestamp - triggerLastRelease;
        long long diffpress = timestamp - triggerLastPress;
        if (gunStatus == 9 && diffpress < max_time_lastPress)
        {
          output_signal = "gunshot";
          triggerLastPress = 0;
        }
        else if (gunStatus != 9 && diffrlz < max_time_lastPress)
        {
          output_signal = "gunshot";
          triggerLastRelease = 0;
        }
        lastOther1 = gunStatus;
      }
    }

    // Mad Dog McCree - Gunslinger Pack (USA)
    if (title == "RQ5E5G")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x803AE899);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x803AE89B);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }

    //Remington Great American Bird Hunt (USA)
    //Note : Not perfect, use cooldown instead of ammo to track gunshot
    if (title == "SBHEFP")
    {
      int cooldown = 0;
      float reload = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_cooldown = PowerPC::MMU::HostTryReadU8(guard, 0x812B75DB);
          if (!blr_cooldown)
            valid_query = false;
          else
            cooldown = blr_cooldown->value;

          auto blr_reload = PowerPC::MMU::HostTryReadF32(guard, 0x812B75E4);
          if (!blr_reload)
            valid_query = false;
          else
            reload = blr_reload->value;
        }
        if (m_index == 1)
        {
          auto blr_cooldown = PowerPC::MMU::HostTryReadU8(guard, 0x812B7C9B);
          if (!blr_cooldown)
            valid_query = false;
          else
            cooldown = blr_cooldown->value;

          auto blr_reload = PowerPC::MMU::HostTryReadF32(guard, 0x812B7CA7);
          if (!blr_reload)
            valid_query = false;
          else
            reload = blr_reload->value;
        }
        // DEBUG_LOG_FMT(ACHIEVEMENTS, "valid= {} {} {}", (int)valid_query, weaponType, ammoCount);
      }
      if (valid_query)
      {
        if (cooldown == 0 && reload == 0)
        {
          long long diff = timestamp - triggerLastPress;
          long long diff2 = timestamp - LastGunshotPress;
          // DEBUG_LOG_FMT(ACHIEVEMENTS, "diff {}", diff);
          if (diff < max_time_lastPress && diff2 > 300000)
          {
            LastGunshotPress = timestamp;
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
        }
        else
        {
          LastGunshotPress = 0;
        }
      }
    }

    //Remington Super Slam Hunting - Africa (USA)
    // Note : Not perfect, use cooldown instead of ammo to track gunshot
    if (title == "SS7EFP")
    {
      int cooldown = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_cooldown = PowerPC::MMU::HostTryReadU8(guard, 0x802ECD41);
          if (!blr_cooldown)
            valid_query = false;
          else
            cooldown = blr_cooldown->value;
        }
        if (m_index == 1)
        {
          auto blr_cooldown = PowerPC::MMU::HostTryReadU8(guard, 0x802ECD6F);
          if (!blr_cooldown)
            valid_query = false;
          else
            cooldown = blr_cooldown->value;
        }
        // DEBUG_LOG_FMT(ACHIEVEMENTS, "valid= {} {} {}", (int)valid_query, weaponType, ammoCount);
      }
      if (valid_query)
      {
        if (cooldown == 0)
        {
          long long diff = timestamp - triggerLastPress;
          long long diff2 = timestamp - LastGunshotPress;
          // DEBUG_LOG_FMT(ACHIEVEMENTS, "diff {}", diff);
          if (diff < max_time_lastPress && diff2 > 300000)
          {
            LastGunshotPress = timestamp;
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
        }
        else
        {
          LastGunshotPress = 0;
        }
      }
    }

    //Resident Evil - The Darkside Chronicles (USA)
    if (title == "SBDE08")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8106C7FF);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8106FFBF);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        // NOTICE_LOG_FMT(ACHIEVEMENTS, "valid= {} {}", m_index, ammoCount);
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo)
        {
          if (triggerIsActive)
          {
            output_signal = "gunshot";
          }
        }
        lastAmmo = ammoCount;
      }
    }

    //Resident Evil - The Umbrella Chronicles (USA)
    if (title == "RBUE08")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x804B779B);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x804B77BF);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo)
        {
          if (triggerIsActive)
          {
            output_signal = "gunshot";
          }
        }
        lastAmmo = ammoCount;
      }
    }

    //Target - Terror (USA)
    if (title == "RGDEA4")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8025A55F);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8025A55F);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo)
        {
          if (triggerIsActive)
          {
            output_signal = "gunshot";
          }
        }
        lastAmmo = ammoCount;
      }
    }

    //Conduit, The (USA)
    if (title == "RCJE8P")
    {
      int ammoCount = 0;
      int ammoCountCharge = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x90D399BB);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x90D399B7);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x90D399B3);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x90D399AF);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x90D399CB);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x90D399C3);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCountCharge = blr_ammo->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo)
        {
          if (triggerIsActive)
          {
            output_signal = "gunshot";
          }
        }
        else
        {
          if (ammoCountCharge < lastCharged)
          {

            long long diffrlz = timestamp - triggerLastRelease;
            if (diffrlz < max_time_lastPress)
            {
              output_signal = "gunshot";
              triggerLastRelease = 0;
            }
          }
        }
        lastCharged = ammoCountCharge;
        lastAmmo = ammoCount;
      }
    }

    //House of the Dead 2 & 3 Return, The (USA)
    if (title == "RHDE8P")  
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x804078ed);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          auto blr_ammo_hd3 = PowerPC::MMU::HostTryReadU8(guard, 0x8042f367);
          if (!blr_ammo_hd3)
            valid_query = false;
          else
            ammoCount += blr_ammo_hd3->value;
        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x80407c5d);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          auto blr_ammo_hd3 = PowerPC::MMU::HostTryReadU8(guard, 0x8042fa0b);
          if (!blr_ammo_hd3)
            valid_query = false;
          else
            ammoCount += blr_ammo_hd3->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount < lastAmmo && triggerIsActive)
        {
          output_signal = "gunshot";
        }
        lastAmmo = ammoCount;
      }
    }

    //House of the Dead, The - Overkill (USA)
    //Note : need to use profile 1 on a specific save file
    if (title == "RHOE8P") 
    {
      int ammoCount = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x814cbe90);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x25f));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
          //NOTICE_LOG_FMT(ACHIEVEMENTS, "Ammo =  {} -> {}", ammoCount, ammoadress);
        }

        if (m_index == 1)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x814cbe94);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x25f));
            if (!blr_ammo)
              valid_query = false;
            else
              ammoCount = blr_ammo->value;
          }
        }

        if (valid_query)
        {
          if (ammoCount < lastAmmo)
          {
            if (triggerIsActive)
            {
              output_signal = "gunshot";
            }
          }
          lastAmmo = ammoCount;
        }

      }
    }

    // Big Buck Hunter Pro (USA)
    if (title == "SBQE4Z")
    {
      int ammoCount = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8035fbd3);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          auto blr_ammo2 = PowerPC::MMU::HostTryReadU8(guard, 0x8035fc4f);
          if (!blr_ammo2)
            valid_query = false;
          else
            ammoCount += blr_ammo2->value;

        }
        if (m_index == 1)
        {
          auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x8035fbd7);
          if (!blr_ammo)
            valid_query = false;
          else
            ammoCount += blr_ammo->value;

          auto blr_ammo2 = PowerPC::MMU::HostTryReadU8(guard, 0x8035fc4b);
          if (!blr_ammo2)
            valid_query = false;
          else
            ammoCount += blr_ammo2->value;
        }
      }
      if (valid_query)
      {
        if (ammoCount > lastAmmo)
        {
          long long diffrlz = timestamp - triggerLastRelease;
          if (triggerIsActive || diffrlz < max_time_lastPress)
          {
            output_signal = "gunshot";
            triggerLastRelease = 0;
          }
        }
        lastAmmo = ammoCount;
      }
    }

    
    // Rayman Raving Rabbids (USA) (Rev 2)
    if (title == "RRBE41")
    {
      int outOfAmmo = 0;
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x806b59a4);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x21D));
            if (!blr_ammo)
              valid_query = false;
            else
              outOfAmmo = blr_ammo->value;
          }
        }
        if (m_index == 1)
        {
          long ammoadress = 0;
          auto blr_ammo_address = PowerPC::MMU::HostTryReadU32(guard, 0x806b59a4);
          if (!blr_ammo_address)
            valid_query = false;
          else
            ammoadress = blr_ammo_address->value;

          if (valid_query)
          {
            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, (ammoadress + 0x225));
            if (!blr_ammo)
              valid_query = false;
            else
              outOfAmmo = blr_ammo->value;
          }
        }
      }
      if (valid_query)
      {
        if (outOfAmmo == 0)
        {
          long long diffrlz = timestamp - triggerLastPress;
          if (diffrlz < max_time_lastPress)
          {
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
        }
      }
    }
    
    //if (lastActiveGame == "RM2E69")  //Medal of Honor HERO 2 (usa)
    if (title == "RM2E69")
    {
      int outOfAmmo = 0;
      int max_player = 1;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
        Core::CPUThreadGuard guard(Core::System::GetInstance());

        if (m_index == 0)
        {

            auto blr_ammo = PowerPC::MMU::HostTryReadU8(guard, 0x929338c8);
            if (!blr_ammo)
              valid_query = false;
            else
              outOfAmmo = blr_ammo->value;

        }
      }

      if (fullAutoActive && (!triggerIsActive || outOfAmmo == 1))
      {
        output_signal = "machinegun_off";
        fullAutoActive = false;
      }

      if (valid_query)
      {
        if (outOfAmmo == 0)
        {
          long long diffrlz = timestamp - triggerLastPress;
          if (triggerIsActive && !fullAutoActive && diffrlz < max_time_lastPress)
          {
            fullAutoActive = true;
            triggerLastPress = 0;
            output_signal = "machinegun_on:160";
          }
        }
      }
    }

    //Failed Recoil List :

    //GunBlade
    if (title == "SQDE8P" || title == "SQDP8P")
    {
      int max_player = 4;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
      }
      if (fullAutoActive && !triggerIsActive)
      {
        output_signal = "machinegun_off";
        fullAutoActive = false;
      }
      if (triggerIsActive && !fullAutoActive)
      {
        fullAutoActive = true;
        triggerLastPress = 0;
        output_signal = "machinegun_on:160";
      }
    }

    // Rayman Raving Rabbids 2 PAL
    if (title == "RY2E41" || title == "RY2J41" || title == "RY2K41" || title == "RY2P41" || title == "RY2R41")
    {
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
        valid_query = true;
      }
      if (valid_query)
      {
          long long diffrlz = timestamp - triggerLastPress;
          if (diffrlz < max_time_lastPress)
          {
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
      }
    }

    //Rayman Raving Rabbids TV Party
    if (title == "RY3E41" || title == "RY3J41" || title == "RY3K41" || title == "RY3P41")
    {
      int max_player = 2;

      if (m_index <= max_player - 1)
      {
          valid_query = true;
      }
      if (valid_query)
      {
          long long diffrlz = timestamp - triggerLastPress;
          if (diffrlz < max_time_lastPress)
          {
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
      }
    }

    //Cocoto Magic Circus
    if (title == "RMRE5Z" || title == "RMRPNK" || title == "RMRXNK")
    {
      int max_player = 4;

      if (m_index <= max_player - 1)
      {
          valid_query = true;
      }
      if (valid_query)
      {
          long long diffrlz = timestamp - triggerLastPress;
          if (diffrlz < max_time_lastPress)
          {
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
      }
    }

    //Dino Strike
    if (title == "SJUE20")
    {
      int max_player = 4;

      if (m_index <= max_player - 1)
      {
          valid_query = true;
      }
      if (valid_query)
      {
          long long diffrlz = timestamp - triggerLastPress;
          if (diffrlz < max_time_lastPress)
          {
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
      }
    }

    //Martian Panic
    if (title == "RQ7E20")
    {
      int max_player = 4;

      if (m_index <= max_player - 1)
      {
          valid_query = true;
      }
      if (valid_query)
      {
          long long diffrlz = timestamp - triggerLastPress;
          if (diffrlz < max_time_lastPress)
          {
            output_signal = "gunshot";
            triggerLastPress = 0;
          }
      }
    }

    bool doRecoil = false;
    if (output_signal != "")
    {
      if (output_signal == "gunshot")
      {
          nextGunShot = 0;
          fullAutoDelay = 0;
          queueSizeGunshot = 0;
          multishotDelay = 0;
          doRecoil = true;
      }
      if (output_signal.starts_with("multishot:"))
      {
          size_t firstColonPos = output_signal.find(':');
          size_t secondColonPos = output_signal.find(':', firstColonPos + 1);

          std::string num1Str =
              output_signal.substr(firstColonPos + 1, secondColonPos - firstColonPos - 1);
          std::string num2Str = output_signal.substr(secondColonPos + 1);

          int numberofshot = std::stoi(num1Str);
          int delayshot = std::stoi(num2Str);


          delayshot *= 1000;
          nextGunShot = timestamp + delayshot;
          fullAutoDelay = 0;
          queueSizeGunshot = numberofshot - 1;
          multishotDelay = delayshot;
          doRecoil = true;
      }
      if (output_signal.starts_with("machinegun_on:"))
      {
          size_t colonPos = output_signal.find(':');
          std::string valueStr = output_signal.substr(colonPos + 1);
          int delayshot = std::stoi(valueStr);

          delayshot *= 1000;
          nextGunShot = timestamp + delayshot;
          fullAutoDelay = delayshot;
          queueSizeGunshot = 0;
          delayshot = 0;
          doRecoil = true;
      }
      if (output_signal == "machinegun_off")
      {
          nextGunShot = 0;
          fullAutoDelay = 0;
      }
    }
    else
    {
      if (queueSizeGunshot > 0 && timestamp > nextGunShot)
      {
          doRecoil = true;
          queueSizeGunshot--;
          if (queueSizeGunshot > 0)
          {
            nextGunShot = timestamp + multishotDelay;
          }
      }
      if (fullAutoDelay > 0 && timestamp > nextGunShot)
      {
          doRecoil = true;
          nextGunShot = timestamp + fullAutoDelay;
      }
    }

    if (doRecoil)
    {
      lastGunShot = timestamp;
      NOTICE_LOG_FMT(ACHIEVEMENTS, "GUN {} : {}", m_index + 1, output_signal);
      if (serialPort != INVALID_HANDLE_VALUE)
      {
          Wiimote::SendComMessage("F0x2x0x");
      }
      MameHookerProxy::GetInstance().Gunshot(m_index);
    }

    /*
    if (output_signal != "")
    {
      NOTICE_LOG_FMT(ACHIEVEMENTS, "GUN {} : {}", m_index + 1, output_signal);
      if (output_signal == "gunshot")
      {
        if (serialPort != INVALID_HANDLE_VALUE)
        {
          Wiimote::SendComMessage("F0x2x0x");
        }
        MameHookerProxy::GetInstance().Gunshot(m_index);
      }

    }
    */

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  DEBUG_LOG_FMT(ACHIEVEMENTS, "THREAD {} : Thread fin", m_index);
  if (serialPort != INVALID_HANDLE_VALUE)
  {
    Wiimote::SendComMessage("E");
    CloseHandle(serialPort);
    serialPort = INVALID_HANDLE_VALUE;
  }

}
Wiimote::Wiimote(const unsigned int index) : m_index(index), m_bt_device_index(index)
{
  fastPointer = Config::Get(Config::MAIN_USE_FAST_POINTER);
  quitThread = false;
  myThread = new std::thread(&Wiimote::threadOutputs, this);

  using Translatability = ControllerEmu::Translatability;

  // Buttons
  groups.emplace_back(m_buttons = new ControllerEmu::Buttons(BUTTONS_GROUP));
  for (auto& named_button : {A_BUTTON, B_BUTTON, ONE_BUTTON, TWO_BUTTON, MINUS_BUTTON, PLUS_BUTTON})
  {
    m_buttons->AddInput(Translatability::DoNotTranslate, named_button);
  }
  m_buttons->AddInput(Translatability::DoNotTranslate, HOME_BUTTON, "HOME");

  // D-Pad
  groups.emplace_back(m_dpad = new ControllerEmu::Buttons(DPAD_GROUP));
  for (const char* named_direction : named_directions)
  {
    m_dpad->AddInput(Translatability::Translate, named_direction);
  }

  // i18n: "Point" refers to the action of pointing a Wii Remote.
  groups.emplace_back(m_ir = new ControllerEmu::Cursor(IR_GROUP, _trans("Point")));
  groups.emplace_back(m_shake = new ControllerEmu::Shake(_trans("Shake")));
  groups.emplace_back(m_tilt = new ControllerEmu::Tilt(_trans("Tilt")));
  groups.emplace_back(m_swing = new ControllerEmu::Force(_trans("Swing")));

  groups.emplace_back(m_imu_ir = new ControllerEmu::IMUCursor("IMUIR", _trans("Point")));
  const auto fov_default =
      Common::DVec2(CameraLogic::CAMERA_FOV_X, CameraLogic::CAMERA_FOV_Y) / MathUtil::TAU * 360;
  m_imu_ir->AddSetting(&m_fov_x_setting,
                       // i18n: FOV stands for "Field of view".
                       {_trans("Horizontal FOV"),
                        // i18n: The symbol/abbreviation for degrees (unit of angular measure).
                        _trans("°"),
                        // i18n: Refers to emulated wii remote camera properties.
                        _trans("Camera field of view (affects sensitivity of pointing).")},
                       fov_default.x, 0.01, 180);
  m_imu_ir->AddSetting(&m_fov_y_setting,
                       // i18n: FOV stands for "Field of view".
                       {_trans("Vertical FOV"),
                        // i18n: The symbol/abbreviation for degrees (unit of angular measure).
                        _trans("°"),
                        // i18n: Refers to emulated wii remote camera properties.
                        _trans("Camera field of view (affects sensitivity of pointing).")},
                       fov_default.y, 0.01, 180);

  groups.emplace_back(m_imu_accelerometer = new ControllerEmu::IMUAccelerometer(
                          ACCELEROMETER_GROUP, _trans("Accelerometer")));
  groups.emplace_back(m_imu_gyroscope =
                          new ControllerEmu::IMUGyroscope(GYROSCOPE_GROUP, _trans("Gyroscope")));

  // Hotkeys
  groups.emplace_back(m_hotkeys = new ControllerEmu::ModifySettingsButton(_trans("Hotkeys")));
  // hotkeys to temporarily modify the Wii Remote orientation (sideways, upright)
  // this setting modifier is toggled
  m_hotkeys->AddInput(_trans("Sideways Toggle"), true);
  m_hotkeys->AddInput(_trans("Upright Toggle"), true);
  // this setting modifier is not toggled
  m_hotkeys->AddInput(_trans("Sideways Hold"), false);
  m_hotkeys->AddInput(_trans("Upright Hold"), false);

  // Extension
  groups.emplace_back(m_attachments = new ControllerEmu::Attachments(_trans("Extension")));
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::None>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::Nunchuk>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::Classic>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::Guitar>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::Drums>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::Turntable>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::UDrawTablet>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::DrawsomeTablet>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::TaTaCon>());
  m_attachments->AddAttachment(std::make_unique<WiimoteEmu::Shinkansen>());

  m_attachments->AddSetting(&m_motion_plus_setting, {_trans("Attach MotionPlus")}, true);

  // Rumble
  groups.emplace_back(m_rumble = new ControllerEmu::ControlGroup(_trans("Rumble")));
  m_rumble->AddOutput(Translatability::Translate, _trans("Motor"));

  // Options
  groups.emplace_back(m_options = new ControllerEmu::ControlGroup(_trans("Options")));

  m_options->AddSetting(&m_speaker_logic.m_speaker_pan_setting,
                        {_trans("Speaker Pan"),
                         // i18n: The percent symbol.
                         _trans("%")},
                        0, -100, 100);

  m_options->AddSetting(&m_battery_setting,
                        {_trans("Battery"),
                         // i18n: The percent symbol.
                         _trans("%")},
                        95, 0, 100);

  // Note: "Upright" and "Sideways" options can be enabled at the same time which produces an
  // orientation where the wiimote points towards the left with the buttons towards you.
  m_options->AddSetting(&m_upright_setting,
                        {UPRIGHT_OPTION, nullptr, nullptr, _trans("Upright Wii Remote")}, false);

  m_options->AddSetting(&m_sideways_setting,
                        {SIDEWAYS_OPTION, nullptr, nullptr, _trans("Sideways Wii Remote")}, false);


    


  Reset();

  m_config_changed_callback_id = Config::AddConfigChangedCallback([this] { RefreshConfig(); });
  RefreshConfig();
}

Wiimote::~Wiimote()
{
  if (myThread != nullptr)
  {
    quitThread = true;
    myThread->join();
  }
  Config::RemoveConfigChangedCallback(m_config_changed_callback_id);
}

void Wiimote::SendComMessage(const std::string& message)
{
  if (serialPort != INVALID_HANDLE_VALUE)
  {
    DWORD bytesWritten;
    DWORD messageLength = static_cast<DWORD>(message.length());
    WriteFile(serialPort, message.c_str(), messageLength, &bytesWritten, NULL);
  }
}

std::string Wiimote::GetName() const
{
  if (m_index == WIIMOTE_BALANCE_BOARD)
    return "BalanceBoard";
  return fmt::format("Wiimote{}", 1 + m_index);
}

InputConfig* Wiimote::GetConfig() const
{
  return ::Wiimote::GetConfig();
}

ControllerEmu::ControlGroup* Wiimote::GetWiimoteGroup(WiimoteGroup group) const
{
  switch (group)
  {
  case WiimoteGroup::Buttons:
    return m_buttons;
  case WiimoteGroup::DPad:
    return m_dpad;
  case WiimoteGroup::Shake:
    return m_shake;
  case WiimoteGroup::Point:
    return m_ir;
  case WiimoteGroup::Tilt:
    return m_tilt;
  case WiimoteGroup::Swing:
    return m_swing;
  case WiimoteGroup::Rumble:
    return m_rumble;
  case WiimoteGroup::Attachments:
    return m_attachments;
  case WiimoteGroup::Options:
    return m_options;
  case WiimoteGroup::Hotkeys:
    return m_hotkeys;
  case WiimoteGroup::IMUAccelerometer:
    return m_imu_accelerometer;
  case WiimoteGroup::IMUGyroscope:
    return m_imu_gyroscope;
  case WiimoteGroup::IMUPoint:
    return m_imu_ir;
  default:
    ASSERT(false);
    return nullptr;
  }
}

ControllerEmu::ControlGroup* Wiimote::GetNunchukGroup(NunchukGroup group) const
{
  return static_cast<Nunchuk*>(m_attachments->GetAttachmentList()[ExtensionNumber::NUNCHUK].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetClassicGroup(ClassicGroup group) const
{
  return static_cast<Classic*>(m_attachments->GetAttachmentList()[ExtensionNumber::CLASSIC].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetGuitarGroup(GuitarGroup group) const
{
  return static_cast<Guitar*>(m_attachments->GetAttachmentList()[ExtensionNumber::GUITAR].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetDrumsGroup(DrumsGroup group) const
{
  return static_cast<Drums*>(m_attachments->GetAttachmentList()[ExtensionNumber::DRUMS].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetTurntableGroup(TurntableGroup group) const
{
  return static_cast<Turntable*>(
             m_attachments->GetAttachmentList()[ExtensionNumber::TURNTABLE].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetUDrawTabletGroup(UDrawTabletGroup group) const
{
  return static_cast<UDrawTablet*>(
             m_attachments->GetAttachmentList()[ExtensionNumber::UDRAW_TABLET].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetDrawsomeTabletGroup(DrawsomeTabletGroup group) const
{
  return static_cast<DrawsomeTablet*>(
             m_attachments->GetAttachmentList()[ExtensionNumber::DRAWSOME_TABLET].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetTaTaConGroup(TaTaConGroup group) const
{
  return static_cast<TaTaCon*>(m_attachments->GetAttachmentList()[ExtensionNumber::TATACON].get())
      ->GetGroup(group);
}

ControllerEmu::ControlGroup* Wiimote::GetShinkansenGroup(ShinkansenGroup group) const
{
  return static_cast<Shinkansen*>(
             m_attachments->GetAttachmentList()[ExtensionNumber::SHINKANSEN].get())
      ->GetGroup(group);
}

bool Wiimote::ProcessExtensionPortEvent()
{
  // WiiBrew: Following a connection or disconnection event on the Extension Port,
  // data reporting is disabled and the Data Reporting Mode must be reset before new data can
  // arrive.
  if (m_extension_port.IsDeviceConnected() == m_status.extension)
    return false;

  // FYI: This happens even during a read request which continues after the status report is sent.
  m_reporting_mode = InputReportID::ReportDisabled;

  DEBUG_LOG_FMT(WIIMOTE, "Sending status report due to extension status change.");

  HandleRequestStatus(OutputReportRequestStatus{});

  return true;
}

void Wiimote::UpdateButtonsStatus(const DesiredWiimoteState& target_state)
{
  m_status.buttons.hex = target_state.buttons.hex & ButtonData::BUTTON_MASK;
}

void Wiimote::BuildDesiredWiimoteState(DesiredWiimoteState* target_state,
                                       SensorBarState sensor_bar_state)
{
  // Hotkey / settings modifier
  // Data is later accessed in IsSideways and IsUpright
  m_hotkeys->UpdateState();

  // Update our motion simulations.
  StepDynamics();

  // Fetch pressed buttons from user input.
  target_state->buttons.hex = 0;
  m_buttons->GetState(&target_state->buttons.hex, button_bitmasks, m_input_override_function);
  m_dpad->GetState(&target_state->buttons.hex,
                   IsSideways() ? dpad_sideways_bitmasks : dpad_bitmasks,
                   m_input_override_function);

  if (target_state->buttons.b)
  {
    if (!triggerIsActive)
    {
      triggerIsActive = true;
      MameHookerProxy::GetInstance().SendState("TriggerPress_P" + std::to_string(m_index + 1), 1);
      triggerLastPress = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count();
      triggerLastPressNoReset = triggerLastPress;
    }
  }
  else
  {
    if (triggerIsActive)
    {
      triggerIsActive = false;
      MameHookerProxy::GetInstance().SendState("TriggerPress_P" + std::to_string(m_index + 1), 0);
      triggerLastRelease = std::chrono::duration_cast<std::chrono::microseconds>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
    }
  }
  // Calculate accelerometer state.
  // Calibration values are 8-bit but we want 10-bit precision, so << 2.
  target_state->acceleration =
      ConvertAccelData(GetTotalAcceleration(), ACCEL_ZERO_G << 2, ACCEL_ONE_G << 2);

  // Calculate IR camera state.
  if (sensor_bar_state == SensorBarState::Enabled)
  {
    target_state->camera_points = CameraLogic::GetCameraPoints(
        GetTotalTransformation(),
        Common::Vec2(m_fov_x_setting.GetValue(), m_fov_y_setting.GetValue()) / 360 *
            float(MathUtil::TAU));
  }
  else
  {
    // If the sensor bar is off the camera will see no LEDs and return 0xFFs.
    target_state->camera_points = DesiredWiimoteState::DEFAULT_CAMERA;
  }

  // Calculate MotionPlus state.
  if (m_motion_plus_setting.GetValue())
    target_state->motion_plus = MotionPlus::GetGyroscopeData(GetTotalAngularVelocity());
  else
    target_state->motion_plus = std::nullopt;

  // Build Extension state.
  // This also allows the extension to perform any regular duties it may need.
  // (e.g. Nunchuk motion simulation step)
  static_cast<Extension*>(
      m_attachments->GetAttachmentList()[m_attachments->GetSelectedAttachment()].get())
      ->BuildDesiredExtensionState(&target_state->extension);
}

u8 Wiimote::GetWiimoteDeviceIndex() const
{
  return m_bt_device_index;
}

void Wiimote::SetWiimoteDeviceIndex(u8 index)
{
  m_bt_device_index = index;
}

// This is called every ::Wiimote::UPDATE_FREQ (200hz)
void Wiimote::PrepareInput(WiimoteEmu::DesiredWiimoteState* target_state,
                           SensorBarState sensor_bar_state)
{
  const auto lock = GetStateLock();
  BuildDesiredWiimoteState(target_state, sensor_bar_state);
}

void Wiimote::Update(const WiimoteEmu::DesiredWiimoteState& target_state)
{
  // Update buttons in the status struct which is sent in 99% of input reports.
  UpdateButtonsStatus(target_state);

  // If a new extension is requested in the GUI the change will happen here.
  HandleExtensionSwap(static_cast<ExtensionNumber>(target_state.extension.data.index()),
                      target_state.motion_plus.has_value());

  // Prepare input data of the extension for reading.
  GetActiveExtension()->Update(target_state.extension);

  if (m_is_motion_plus_attached)
  {
    // M+ has some internal state that must processed.
    m_motion_plus.Update(target_state.extension);
  }

  // Returns true if a report was sent.
  if (ProcessExtensionPortEvent())
  {
    // Extension port event occurred.
    // Don't send any other reports.
    return;
  }

  if (ProcessReadDataRequest())
  {
    // Read requests suppress normal input reports
    // Don't send any other reports
    return;
  }

  SendDataReport(target_state);
}

void Wiimote::SendDataReport(const DesiredWiimoteState& target_state)
{
  auto& movie = Core::System::GetInstance().GetMovie();
  movie.SetPolledDevice();

  if (InputReportID::ReportDisabled == m_reporting_mode)
  {
    // The wiimote is in this disabled after an extension change.
    // Input reports are not sent, even on button change.
    return;
  }

  if (InputReportID::ReportCore == m_reporting_mode && !m_reporting_continuous)
  {
    // TODO: we only need to send a report if the data changed when m_reporting_continuous is
    // disabled. It's probably only sensible to check this with REPORT_CORE
  }

  DataReportBuilder rpt_builder(m_reporting_mode);

  if (movie.IsPlayingInput() && movie.PlayWiimote(m_bt_device_index, rpt_builder,
                                                  m_active_extension, GetExtensionEncryptionKey()))
  {
    // Update buttons in status struct from movie:
    rpt_builder.GetCoreData(&m_status.buttons);
  }
  else
  {
    // Core buttons:
    if (rpt_builder.HasCore())
    {
      rpt_builder.SetCoreData(m_status.buttons);
    }

    // Acceleration:
    if (rpt_builder.HasAccel())
    {
      rpt_builder.SetAccelData(target_state.acceleration);
    }

    // IR Camera:
    if (rpt_builder.HasIR())
    {
      // Note: Camera logic currently contains no changing state so we can just update it here.
      // If that changes this should be moved to Wiimote::Update();
      m_camera_logic.Update(target_state.camera_points);

      // The real wiimote reads camera data from the i2c bus starting at offset 0x37:
      const u8 camera_data_offset =
          CameraLogic::REPORT_DATA_OFFSET + rpt_builder.GetIRDataFormatOffset();

      u8* ir_data = rpt_builder.GetIRDataPtr();
      const u8 ir_size = rpt_builder.GetIRDataSize();

      if (ir_size != m_i2c_bus.BusRead(CameraLogic::I2C_ADDR, camera_data_offset, ir_size, ir_data))
      {
        // This happens when IR reporting is enabled but the camera hardware is disabled.
        // It commonly occurs when changing IR sensitivity.
        std::fill_n(ir_data, ir_size, u8(0xff));
      }
    }

    // Extension port:
    if (rpt_builder.HasExt())
    {
      // Prepare extension input first as motion-plus may read from it.
      // This currently happens in Wiimote::Update();
      // TODO: Separate extension input data preparation from Update.
      // GetActiveExtension()->PrepareInput();

      if (m_is_motion_plus_attached)
      {
        // TODO: Make input preparation triggered by bus read.
        m_motion_plus.PrepareInput(target_state.motion_plus.has_value() ?
                                       target_state.motion_plus.value() :
                                       MotionPlus::GetDefaultGyroscopeData());
      }

      u8* ext_data = rpt_builder.GetExtDataPtr();
      const u8 ext_size = rpt_builder.GetExtDataSize();

      if (ext_size != m_i2c_bus.BusRead(ExtensionPort::REPORT_I2C_SLAVE,
                                        ExtensionPort::REPORT_I2C_ADDR, ext_size, ext_data))
      {
        // Real wiimote seems to fill with 0xff on failed bus read
        std::fill_n(ext_data, ext_size, u8(0xff));
      }
    }
  }

  movie.CheckWiimoteStatus(m_bt_device_index, rpt_builder, m_active_extension,
                           GetExtensionEncryptionKey());

  // Send the report:
  InterruptDataInputCallback(rpt_builder.GetDataPtr(), rpt_builder.GetDataSize());

  // The interleaved reporting modes toggle back and forth:
  if (InputReportID::ReportInterleave1 == m_reporting_mode)
    m_reporting_mode = InputReportID::ReportInterleave2;
  else if (InputReportID::ReportInterleave2 == m_reporting_mode)
    m_reporting_mode = InputReportID::ReportInterleave1;
}

ButtonData Wiimote::GetCurrentlyPressedButtons()
{
  const auto lock = GetStateLock();

  ButtonData buttons{};
  m_buttons->GetState(&buttons.hex, button_bitmasks, m_input_override_function);
  m_dpad->GetState(&buttons.hex, IsSideways() ? dpad_sideways_bitmasks : dpad_bitmasks,
                   m_input_override_function);

  return buttons;
}

void Wiimote::LoadDefaults(const ControllerInterface& ciface)
{
  EmulatedController::LoadDefaults(ciface);

#ifdef ANDROID
  // Rumble
  m_rumble->SetControlExpression(0, "`Android/0/Device Sensors:Motor 0`");

  // Motion Source
  m_imu_accelerometer->SetControlExpression(0, "`Android/0/Device Sensors:Accel Up`");
  m_imu_accelerometer->SetControlExpression(1, "`Android/0/Device Sensors:Accel Down`");
  m_imu_accelerometer->SetControlExpression(2, "`Android/0/Device Sensors:Accel Left`");
  m_imu_accelerometer->SetControlExpression(3, "`Android/0/Device Sensors:Accel Right`");
  m_imu_accelerometer->SetControlExpression(4, "`Android/0/Device Sensors:Accel Forward`");
  m_imu_accelerometer->SetControlExpression(5, "`Android/0/Device Sensors:Accel Backward`");
  m_imu_gyroscope->SetControlExpression(0, "`Android/0/Device Sensors:Gyro Pitch Up`");
  m_imu_gyroscope->SetControlExpression(1, "`Android/0/Device Sensors:Gyro Pitch Down`");
  m_imu_gyroscope->SetControlExpression(2, "`Android/0/Device Sensors:Gyro Roll Left`");
  m_imu_gyroscope->SetControlExpression(3, "`Android/0/Device Sensors:Gyro Roll Right`");
  m_imu_gyroscope->SetControlExpression(4, "`Android/0/Device Sensors:Gyro Yaw Left`");
  m_imu_gyroscope->SetControlExpression(5, "`Android/0/Device Sensors:Gyro Yaw Right`");
#else
// Buttons
#if defined HAVE_X11 && HAVE_X11
  // A
  m_buttons->SetControlExpression(0, "`Click 1`");
  // B
  m_buttons->SetControlExpression(1, "`Click 3`");
#elif defined(__APPLE__)
  // A
  m_buttons->SetControlExpression(0, "`Left Click`");
  // B
  m_buttons->SetControlExpression(1, "`Right Click`");
#else
  // A
  m_buttons->SetControlExpression(0, "`Click 0`");
  // B
  m_buttons->SetControlExpression(1, "`Click 1`");
#endif
  m_buttons->SetControlExpression(2, "`1`");     // 1
  m_buttons->SetControlExpression(3, "`2`");     // 2
  m_buttons->SetControlExpression(4, "Q");       // -
  m_buttons->SetControlExpression(5, "E");       // +

#ifdef _WIN32
  m_buttons->SetControlExpression(6, "RETURN");  // Home
#else
  // Home
  m_buttons->SetControlExpression(6, "Return");
#endif

  // Shake
  for (int i = 0; i < 3; ++i)
#ifdef __APPLE__
    m_shake->SetControlExpression(i, "`Middle Click`");
#else
    m_shake->SetControlExpression(i, "`Click 2`");
#endif

  // Pointing (IR)
  m_ir->SetControlExpression(0, "`Cursor Y-`");
  m_ir->SetControlExpression(1, "`Cursor Y+`");
  m_ir->SetControlExpression(2, "`Cursor X-`");
  m_ir->SetControlExpression(3, "`Cursor X+`");

// DPad
#ifdef _WIN32
  m_dpad->SetControlExpression(0, "UP");     // Up
  m_dpad->SetControlExpression(1, "DOWN");   // Down
  m_dpad->SetControlExpression(2, "LEFT");   // Left
  m_dpad->SetControlExpression(3, "RIGHT");  // Right
#elif __APPLE__
  m_dpad->SetControlExpression(0, "`Up Arrow`");     // Up
  m_dpad->SetControlExpression(1, "`Down Arrow`");   // Down
  m_dpad->SetControlExpression(2, "`Left Arrow`");   // Left
  m_dpad->SetControlExpression(3, "`Right Arrow`");  // Right
#else
  m_dpad->SetControlExpression(0, "Up");     // Up
  m_dpad->SetControlExpression(1, "Down");   // Down
  m_dpad->SetControlExpression(2, "Left");   // Left
  m_dpad->SetControlExpression(3, "Right");  // Right
#endif

  // Motion Source
  m_imu_accelerometer->SetControlExpression(0, "`Accel Up`");
  m_imu_accelerometer->SetControlExpression(1, "`Accel Down`");
  m_imu_accelerometer->SetControlExpression(2, "`Accel Left`");
  m_imu_accelerometer->SetControlExpression(3, "`Accel Right`");
  m_imu_accelerometer->SetControlExpression(4, "`Accel Forward`");
  m_imu_accelerometer->SetControlExpression(5, "`Accel Backward`");
  m_imu_gyroscope->SetControlExpression(0, "`Gyro Pitch Up`");
  m_imu_gyroscope->SetControlExpression(1, "`Gyro Pitch Down`");
  m_imu_gyroscope->SetControlExpression(2, "`Gyro Roll Left`");
  m_imu_gyroscope->SetControlExpression(3, "`Gyro Roll Right`");
  m_imu_gyroscope->SetControlExpression(4, "`Gyro Yaw Left`");
  m_imu_gyroscope->SetControlExpression(5, "`Gyro Yaw Right`");
#endif

  // Enable Nunchuk:
  constexpr ExtensionNumber DEFAULT_EXT = ExtensionNumber::NUNCHUK;
  m_attachments->SetSelectedAttachment(DEFAULT_EXT);
  m_attachments->GetAttachmentList()[DEFAULT_EXT]->LoadDefaults(ciface);
}

Extension* Wiimote::GetNoneExtension() const
{
  return static_cast<Extension*>(m_attachments->GetAttachmentList()[ExtensionNumber::NONE].get());
}

Extension* Wiimote::GetActiveExtension() const
{
  return static_cast<Extension*>(m_attachments->GetAttachmentList()[m_active_extension].get());
}

EncryptionKey Wiimote::GetExtensionEncryptionKey() const
{
  if (ExtensionNumber::NONE == GetActiveExtensionNumber())
    return {};

  return static_cast<EncryptedExtension*>(GetActiveExtension())->ext_key;
}

bool Wiimote::IsSideways() const
{
  const bool sideways_modifier_toggle = m_hotkeys->GetSettingsModifier()[0];
  const bool sideways_modifier_switch = m_hotkeys->GetSettingsModifier()[2];
  return m_sideways_setting.GetValue() ^ sideways_modifier_toggle ^ sideways_modifier_switch;
}

bool Wiimote::IsUpright() const
{
  const bool upright_modifier_toggle = m_hotkeys->GetSettingsModifier()[1];
  const bool upright_modifier_switch = m_hotkeys->GetSettingsModifier()[3];
  return m_upright_setting.GetValue() ^ upright_modifier_toggle ^ upright_modifier_switch;
}

void Wiimote::SetRumble(bool on)
{
  MameHookerProxy::GetInstance().SendState("Rumble_P" + std::to_string(m_index + 1), on ? 1 : 0);
  const auto lock = GetStateLock();
  m_rumble->controls.front()->control_ref->State(on);
}

void Wiimote::RefreshConfig()
{
  m_speaker_logic.SetSpeakerEnabled(Config::Get(Config::MAIN_WIIMOTE_ENABLE_SPEAKER));
}

void Wiimote::StepDynamics()
{
  EmulateSwing(&m_swing_state, m_swing, 1.f / ::Wiimote::UPDATE_FREQ);
  EmulateTilt(&m_tilt_state, m_tilt, 1.f / ::Wiimote::UPDATE_FREQ);
  EmulatePoint(&m_point_state, m_ir, m_input_override_function, 1.f / ::Wiimote::UPDATE_FREQ,
               lastActiveGame, lastRatio, fastPointer);
  EmulateShake(&m_shake_state, m_shake, 1.f / ::Wiimote::UPDATE_FREQ);
  EmulateIMUCursor(&m_imu_cursor_state, m_imu_ir, m_imu_accelerometer, m_imu_gyroscope,
                   1.f / ::Wiimote::UPDATE_FREQ);
}

Common::Vec3 Wiimote::GetAcceleration(Common::Vec3 extra_acceleration) const
{
  Common::Vec3 accel = GetOrientation() * GetTransformation().Transform(
                                              m_swing_state.acceleration + extra_acceleration, 0);

  // Our shake effects have never been affected by orientation. Should they be?
  accel += m_shake_state.acceleration;

  return accel;
}

Common::Vec3 Wiimote::GetAngularVelocity(Common::Vec3 extra_angular_velocity) const
{
  return GetOrientation() * (m_tilt_state.angular_velocity + m_swing_state.angular_velocity +
                             m_point_state.angular_velocity + extra_angular_velocity);
}

Common::Matrix44 Wiimote::GetTransformation(const Common::Matrix33& extra_rotation) const
{
  // Includes positional and rotational effects of:
  // Point, Swing, Tilt, Shake

  // TODO: Think about and clean up matrix order + make nunchuk match.
  return Common::Matrix44::Translate(-m_shake_state.position) *
         Common::Matrix44::FromMatrix33(extra_rotation * GetRotationalMatrix(-m_tilt_state.angle) *
                                        GetRotationalMatrix(-m_point_state.angle) *
                                        GetRotationalMatrix(-m_swing_state.angle)) *
         Common::Matrix44::Translate(-m_swing_state.position - m_point_state.position);
}

Common::Quaternion Wiimote::GetOrientation() const
{
  return Common::Quaternion::RotateZ(float(MathUtil::TAU / -4 * IsSideways())) *
         Common::Quaternion::RotateX(float(MathUtil::TAU / 4 * IsUpright()));
}

std::optional<Common::Vec3> Wiimote::OverrideVec3(const ControllerEmu::ControlGroup* control_group,
                                                  std::optional<Common::Vec3> optional_vec) const
{
  bool has_value = optional_vec.has_value();
  Common::Vec3 vec = has_value ? *optional_vec : Common::Vec3{};

  if (m_input_override_function)
  {
    if (const std::optional<ControlState> x_override = m_input_override_function(
            control_group->name, ControllerEmu::ReshapableInput::X_INPUT_OVERRIDE, vec.x))
    {
      has_value = true;
      vec.x = *x_override;
    }

    if (const std::optional<ControlState> y_override = m_input_override_function(
            control_group->name, ControllerEmu::ReshapableInput::Y_INPUT_OVERRIDE, vec.y))
    {
      has_value = true;
      vec.y = *y_override;
    }

    if (const std::optional<ControlState> z_override = m_input_override_function(
            control_group->name, ControllerEmu::ReshapableInput::Z_INPUT_OVERRIDE, vec.z))
    {
      has_value = true;
      vec.z = *z_override;
    }
  }

  return has_value ? std::make_optional(vec) : std::nullopt;
}

Common::Vec3 Wiimote::OverrideVec3(const ControllerEmu::ControlGroup* control_group,
                                   Common::Vec3 vec) const
{
  return OverrideVec3(control_group, vec, m_input_override_function);
}

Common::Vec3
Wiimote::OverrideVec3(const ControllerEmu::ControlGroup* control_group, Common::Vec3 vec,
                      const ControllerEmu::InputOverrideFunction& input_override_function)
{
  if (input_override_function)
  {
    if (const std::optional<ControlState> x_override = input_override_function(
            control_group->name, ControllerEmu::ReshapableInput::X_INPUT_OVERRIDE, vec.x))
    {
      vec.x = *x_override;
    }

    if (const std::optional<ControlState> y_override = input_override_function(
            control_group->name, ControllerEmu::ReshapableInput::Y_INPUT_OVERRIDE, vec.y))
    {
      vec.y = *y_override;
    }

    if (const std::optional<ControlState> z_override = input_override_function(
            control_group->name, ControllerEmu::ReshapableInput::Z_INPUT_OVERRIDE, vec.z))
    {
      vec.z = *z_override;
    }
  }

  return vec;
}

Common::Vec3 Wiimote::GetTotalAcceleration() const
{
  const Common::Vec3 default_accel = Common::Vec3(0, 0, float(GRAVITY_ACCELERATION));
  const Common::Vec3 accel = m_imu_accelerometer->GetState().value_or(default_accel);

  return OverrideVec3(m_imu_accelerometer, GetAcceleration(accel));
}

Common::Vec3 Wiimote::GetTotalAngularVelocity() const
{
  const Common::Vec3 default_ang_vel = {};
  const Common::Vec3 ang_vel = m_imu_gyroscope->GetState().value_or(default_ang_vel);

  return OverrideVec3(m_imu_gyroscope, GetAngularVelocity(ang_vel));
}

Common::Matrix44 Wiimote::GetTotalTransformation() const
{
  return GetTransformation(Common::Matrix33::FromQuaternion(
      m_imu_cursor_state.rotation *
      Common::Quaternion::RotateX(m_imu_cursor_state.recentered_pitch)));
}

}  // namespace WiimoteEmu
