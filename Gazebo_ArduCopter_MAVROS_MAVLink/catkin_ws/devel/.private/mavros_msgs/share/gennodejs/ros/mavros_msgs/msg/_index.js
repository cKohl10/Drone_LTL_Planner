
"use strict";

let HilGPS = require('./HilGPS.js');
let Vibration = require('./Vibration.js');
let VFR_HUD = require('./VFR_HUD.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let MountControl = require('./MountControl.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let LandingTarget = require('./LandingTarget.js');
let LogData = require('./LogData.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let Param = require('./Param.js');
let HomePosition = require('./HomePosition.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let Altitude = require('./Altitude.js');
let GPSRAW = require('./GPSRAW.js');
let HilControls = require('./HilControls.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let GPSRTK = require('./GPSRTK.js');
let FileEntry = require('./FileEntry.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let ExtendedState = require('./ExtendedState.js');
let Thrust = require('./Thrust.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let RadioStatus = require('./RadioStatus.js');
let BatteryStatus = require('./BatteryStatus.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let ESCStatus = require('./ESCStatus.js');
let PositionTarget = require('./PositionTarget.js');
let Tunnel = require('./Tunnel.js');
let GPSINPUT = require('./GPSINPUT.js');
let CellularStatus = require('./CellularStatus.js');
let ManualControl = require('./ManualControl.js');
let LogEntry = require('./LogEntry.js');
let WaypointReached = require('./WaypointReached.js');
let RCIn = require('./RCIn.js');
let ParamValue = require('./ParamValue.js');
let ESCInfo = require('./ESCInfo.js');
let DebugValue = require('./DebugValue.js');
let VehicleInfo = require('./VehicleInfo.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let StatusText = require('./StatusText.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let RCOut = require('./RCOut.js');
let Waypoint = require('./Waypoint.js');
let HilSensor = require('./HilSensor.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let Trajectory = require('./Trajectory.js');
let TerrainReport = require('./TerrainReport.js');
let State = require('./State.js');
let Mavlink = require('./Mavlink.js');
let ActuatorControl = require('./ActuatorControl.js');
let RTCM = require('./RTCM.js');
let WaypointList = require('./WaypointList.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let CommandCode = require('./CommandCode.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let RTKBaseline = require('./RTKBaseline.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');

module.exports = {
  HilGPS: HilGPS,
  Vibration: Vibration,
  VFR_HUD: VFR_HUD,
  CamIMUStamp: CamIMUStamp,
  MountControl: MountControl,
  OverrideRCIn: OverrideRCIn,
  LandingTarget: LandingTarget,
  LogData: LogData,
  ESCStatusItem: ESCStatusItem,
  Param: Param,
  HomePosition: HomePosition,
  MagnetometerReporter: MagnetometerReporter,
  AttitudeTarget: AttitudeTarget,
  Altitude: Altitude,
  GPSRAW: GPSRAW,
  HilControls: HilControls,
  EstimatorStatus: EstimatorStatus,
  ESCTelemetry: ESCTelemetry,
  ADSBVehicle: ADSBVehicle,
  GPSRTK: GPSRTK,
  FileEntry: FileEntry,
  CameraImageCaptured: CameraImageCaptured,
  ESCInfoItem: ESCInfoItem,
  OnboardComputerStatus: OnboardComputerStatus,
  ExtendedState: ExtendedState,
  Thrust: Thrust,
  GlobalPositionTarget: GlobalPositionTarget,
  RadioStatus: RadioStatus,
  BatteryStatus: BatteryStatus,
  CompanionProcessStatus: CompanionProcessStatus,
  ESCStatus: ESCStatus,
  PositionTarget: PositionTarget,
  Tunnel: Tunnel,
  GPSINPUT: GPSINPUT,
  CellularStatus: CellularStatus,
  ManualControl: ManualControl,
  LogEntry: LogEntry,
  WaypointReached: WaypointReached,
  RCIn: RCIn,
  ParamValue: ParamValue,
  ESCInfo: ESCInfo,
  DebugValue: DebugValue,
  VehicleInfo: VehicleInfo,
  TimesyncStatus: TimesyncStatus,
  StatusText: StatusText,
  PlayTuneV2: PlayTuneV2,
  WheelOdomStamped: WheelOdomStamped,
  RCOut: RCOut,
  Waypoint: Waypoint,
  HilSensor: HilSensor,
  HilStateQuaternion: HilStateQuaternion,
  HilActuatorControls: HilActuatorControls,
  Trajectory: Trajectory,
  TerrainReport: TerrainReport,
  State: State,
  Mavlink: Mavlink,
  ActuatorControl: ActuatorControl,
  RTCM: RTCM,
  WaypointList: WaypointList,
  NavControllerOutput: NavControllerOutput,
  CommandCode: CommandCode,
  OpticalFlowRad: OpticalFlowRad,
  RTKBaseline: RTKBaseline,
  ESCTelemetryItem: ESCTelemetryItem,
};
