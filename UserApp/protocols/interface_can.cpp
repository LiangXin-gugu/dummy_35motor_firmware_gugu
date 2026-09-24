#include "common_inc.h"
#include "configurations.h"
#include <can.h>


extern Motor motor;
extern EncoderCalibrator encoderCalibrator;

CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0x00,
        .ExtId = 0x00,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };


void OnCanCmd(uint8_t _cmd, uint8_t* _data, uint32_t _len)
{
    float tmpF;
    int32_t tmpI;

    switch (_cmd)
    {
        // 0x00~0x0F No Memory CMDs
        case 0x01:  // Enable Motor
            motor.controller->requestMode = (*(uint32_t*) (RxData) == 1) ?
                                            Motor::MODE_COMMAND_VELOCITY : Motor::MODE_STOP;
            break;
        case 0x02:  // Do Calibration
            encoderCalibrator.isTriggered = true;
            break;
        case 0x03:  // Set Current SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
            motor.controller->SetCurrentSetPoint((int32_t) (*(float*) RxData * 1000));
            break;
        case 0x04:  // Set Velocity SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
            }
            motor.controller->SetVelocitySetPoint(
                (int32_t) (*(float*) RxData *
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            break;
        case 0x05:  // Set Position SetPoint
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            printf("SET MOTOR[0x05] POSITION[]\r\n");
            if (_data[4]) // Need Position & Finished ACK
            {
                tmpF = motor.controller->GetPosition();
                auto* b = (unsigned char*) &tmpF;
                for (int i = 0; i < 4; i++)
                    _data[i] = *(b + i);
                _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
                txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
                CAN_Send(&txHeader, _data);
            }
            break;
        case 0x06:  // Set Position with Time
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            motor.controller->SetPositionSetPointWithTime(
                (int32_t) (*(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
                *(float*) (RxData + 4));
            if (_data[4]) // Need Position & Finished ACK
            {
                tmpF = motor.controller->GetPosition();
                auto* b = (unsigned char*) &tmpF;
                for (int i = 0; i < 4; i++)
                    _data[i] = *(b + i);
                _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
                txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
                CAN_Send(&txHeader, _data);
            }
            break;
        case 0x07:  // Set Position with Velocity-Limit
        {
            if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            {
                motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
                motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
            }
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) (RxData + 4) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            motor.controller->SetPositionSetPoint(
                (int32_t) (*(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
            // Always Need Position & Finished ACK
            tmpF = motor.controller->GetPosition();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
            CAN_Send(&txHeader, _data);
        }
            break;

            // 0x10~0x1F CMDs with Memory
        case 0x11:  // Set Node-ID and Store to EEPROM
            boardConfig.canNodeId = *(uint32_t*) (RxData);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x12:  // Set Current-Limit and Store to EEPROM
            motor.config.motionParams.ratedCurrent = (int32_t) (*(float*) RxData * 1000);
            boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x13:  // Set Velocity-Limit and Store to EEPROM
            motor.config.motionParams.ratedVelocity =
                (int32_t) (*(float*) RxData *
                           (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
            boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x14:  // Set Acceleration （and Store to EEPROM）
            tmpF = *(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

            motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
            motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
            motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
            boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x15:  // Apply Home-Position and Store to EEPROM
            motor.controller->ApplyPosAsHomeOffset();
            boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                            motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
            boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x16:  // Set Auto-Enable and Store to EEPROM
            boardConfig.enableMotorOnBoot = (*(uint32_t*) (RxData) == 1);
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x17:  // Set DCE Kp
            motor.config.ctrlParams.dce.kp = *(int32_t*) (RxData);
            boardConfig.dce_kp = motor.config.ctrlParams.dce.kp;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x18:  // Set DCE Kv
            motor.config.ctrlParams.dce.kv = *(int32_t*) (RxData);
            boardConfig.dce_kv = motor.config.ctrlParams.dce.kv;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x19:  // Set DCE Ki
            motor.config.ctrlParams.dce.ki = *(int32_t*) (RxData);
            boardConfig.dce_ki = motor.config.ctrlParams.dce.ki;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x1A:  // Set DCE Kd
            motor.config.ctrlParams.dce.kd = *(int32_t*) (RxData);
            boardConfig.dce_kd = motor.config.ctrlParams.dce.kd;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;
        case 0x1B:  // Set Enable Stall-Protect
            motor.config.ctrlParams.stallProtectSwitch = (*(uint32_t*) (RxData) == 1);
            boardConfig.enableStallProtect = motor.config.ctrlParams.stallProtectSwitch;
            if (_data[4])
                boardConfig.configStatus = CONFIG_COMMIT;
            break;


            // 0x30~0x3F Controller Query CMDs
        case 0x30:  // Get Controller Status (compressed format)
        {
            // Note: Mode_t and State_t are enums (typically int32_t), but we only send
            // the low byte since valid values are always < 256. This is safe because:
            // - Request/Run modes: MODE_STOP=0 to MODE_STEP_DIR=8 (< 256)
            // - States: STATE_NO_CALIB=5 (< 256)
            _data[0] = (uint8_t) motor.controller->requestMode;
            _data[1] = (uint8_t) motor.controller->modeRunning;
            _data[2] = (uint8_t) motor.controller->state;
            _data[3] = _data[4] = _data[5] = _data[6] = _data[7] = 0;  // padding
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x30;
            CAN_Send(&txHeader, _data);
        }
            break;
                
        case 0x31:  // Get DCE Parameters (Low): kp, kv
        {
            int32_t kp = motor.config.ctrlParams.dce.kp;
            int32_t kv = motor.config.ctrlParams.dce.kv;
            auto* b = (unsigned char*) &kp;
            for (int i = 0; i < 4; i++) _data[i] = *(b + i);
            b = (unsigned char*) &kv;
            for (int i = 0; i < 4; i++) _data[i + 4] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x31;
            CAN_Send(&txHeader, _data);
        }
            break;
                
        case 0x32:  // Get DCE Parameters (High): ki, kd
        {
            int32_t ki = motor.config.ctrlParams.dce.ki;
            int32_t kd = motor.config.ctrlParams.dce.kd;
            auto* b = (unsigned char*) &ki;
            for (int i = 0; i < 4; i++) _data[i] = *(b + i);
            b = (unsigned char*) &kd;
            for (int i = 0; i < 4; i++) _data[i + 4] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x32;
            CAN_Send(&txHeader, _data);
        }
            break;
                
        case 0x33:  // Get DCE Output (Low): outputKp, outputKi
        {
            int32_t output_kp = motor.config.ctrlParams.dce.outputKp;
            int32_t output_ki = motor.config.ctrlParams.dce.outputKi;
            auto* b = (unsigned char*) &output_kp;
            for (int i = 0; i < 4; i++) _data[i] = *(b + i);
            b = (unsigned char*) &output_ki;
            for (int i = 0; i < 4; i++) _data[i + 4] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x33;
            CAN_Send(&txHeader, _data);
        }
            break;
                
        case 0x34:  // Get DCE Output (High): outputKd, output
        {
            int32_t output_kd = motor.config.ctrlParams.dce.outputKd;
            int32_t output_total = motor.config.ctrlParams.dce.output;
            auto* b = (unsigned char*) &output_kd;
            for (int i = 0; i < 4; i++) _data[i] = *(b + i);
            b = (unsigned char*) &output_total;
            for (int i = 0; i < 4; i++) _data[i + 4] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x34;
            CAN_Send(&txHeader, _data);
        }
            break;
                
        case 0x35:  // Get Controller Position Info: realPosition, estPosition
        {
            int32_t real_pos = motor.controller->GetRealPosition();
            int32_t est_pos = motor.controller->GetEstPosition();
            auto* b = (unsigned char*) &real_pos;
            for (int i = 0; i < 4; i++) _data[i] = *(b + i);
            b = (unsigned char*) &est_pos;
            for (int i = 0; i < 4; i++) _data[i + 4] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x35;
            CAN_Send(&txHeader, _data);
        }
            break;
                
        case 0x36:  // Get Controller Velocity Info: estVelocity, softVelocity
        {
            int32_t est_vel = motor.controller->GetEstVelocity();
            int32_t soft_vel = motor.controller->GetSoftVelocity();
            auto* b = (unsigned char*) &est_vel;
            for (int i = 0; i < 4; i++) _data[i] = *(b + i);
            b = (unsigned char*) &soft_vel;
            for (int i = 0; i < 4; i++) _data[i + 4] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x36;
            CAN_Send(&txHeader, _data);
        }
            break;
                
        case 0x37:  // Get Controller Soft Position: softPosition, (padding)
        {
            int32_t soft_pos = motor.controller->GetSoftPosition();
            auto* b = (unsigned char*) &soft_pos;
            for (int i = 0; i < 4; i++) _data[i] = *(b + i);
            // Padding bytes not needed
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x37;
            CAN_Send(&txHeader, _data);
        }
            break;
        
            // 0x20~0x2F Inquiry CMDs
        case 0x21: // Get Current
        {
            tmpF = motor.controller->GetFocCurrent();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x21;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x22: // Get Velocity
        {
            tmpF = motor.controller->GetVelocity();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x22;
            CAN_Send(&txHeader, _data);
        }
            break;
        case 0x23: // Get Position
        {
            tmpF = motor.controller->GetPosition();
            auto* b = (unsigned char*) &tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            // Finished ACK
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
            CAN_Send(&txHeader, _data);
//            printf("CAN SEND BACK to NODE[%d]\n", boardConfig.canNodeId );
        }
            break;
        case 0x24: // Get Offset
        {
            tmpI = motor.config.motionParams.encoderHomeOffset;
            auto* b = (unsigned char*) &tmpI;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x24;
            CAN_Send(&txHeader, _data);
        }
            break;

        case 0x25: // Get temperature
        {
            auto* b = (unsigned char*) &boardConfig.motor_temperature;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = 0;
            _data[5] = 0;
            _data[6] = 0;
            _data[7] = 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x25;
            CAN_Send(&txHeader, _data);
        }
            break;

        case 0x7d:  // enable motor temperature watch
            boardConfig.enableTempWatch = true;
            break;
        case 0x7e:  // Erase Configs
            boardConfig.configStatus = CONFIG_RESTORE;
            break;
        case 0x7f:  // Reboot
            HAL_NVIC_SystemReset();
            break;
        default:
            break;
    }

}

