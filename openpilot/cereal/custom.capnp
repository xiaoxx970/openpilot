using Cxx = import "/include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# DO rename the structs
# DON'T change the identifier (e.g. @0x81c2f05a394cf4af)

struct ModularAssistiveDrivingSystem {
  state @0 :ModularAssistiveDrivingSystemState;
  enabled @1 :Bool;
  active @2 :Bool;
  available @3 :Bool;

  enum ModularAssistiveDrivingSystemState {
    disabled @0;
    paused @1;
    enabled @2;
    softDisabling @3;
    overriding @4;
  }
}

struct IntelligentCruiseButtonManagement {
  state @0 :IntelligentCruiseButtonManagementState;
  sendButton @1 :SendButtonState;
  vTarget @2 :Float32;

  enum IntelligentCruiseButtonManagementState {
    inactive @0;      # No button press or default state
    preActive @1;     # Pre-active state before transitioning to increasing or decreasing
    increasing @2;    # Increasing speed
    decreasing @3;    # Decreasing speed
    holding @4;       # Holding steady speed
  }

  enum SendButtonState {
    none @0;
    increase @1;
    decrease @2;
  }
}

# Same struct as Log.RadarState.LeadData
struct LeadData {
  dRel @0 :Float32;
  yRel @1 :Float32;
  vRel @2 :Float32;
  aRel @3 :Float32;
  vLead @4 :Float32;
  dPath @6 :Float32;
  vLat @7 :Float32;
  vLeadK @8 :Float32;
  aLeadK @9 :Float32;
  fcw @10 :Bool;
  status @11 :Bool;
  aLeadTau @12 :Float32;
  modelProb @13 :Float32;
  radar @14 :Bool;
  radarTrackId @15 :Int32 = -1;

  aLeadDEPRECATED @5 :Float32;
}

struct SelfdriveStateSP @0x81c2f05a394cf4af {
  mads @0 :ModularAssistiveDrivingSystem;
  intelligentCruiseButtonManagement @1 :IntelligentCruiseButtonManagement;
  buttonsPressed @2 :UInt16;
  buttonsReleaseToggle @3 :UInt16;

  enum AudibleAlert {
    none @0;

    engage @1;
    disengage @2;
    refuse @3;

    warningSoft @4;
    warningImmediate @5;

    prompt @6;
    promptRepeat @7;
    promptDistracted @8;

    # unused, these are reserved for upstream events so we don't collide
    reserved9 @9;
    reserved10 @10;
    reserved11 @11;
    reserved12 @12;
    reserved13 @13;
    reserved14 @14;
    reserved15 @15;
    reserved16 @16;
    reserved17 @17;
    reserved18 @18;
    reserved19 @19;
    reserved20 @20;
    reserved21 @21;
    reserved22 @22;
    reserved23 @23;
    reserved24 @24;
    reserved25 @25;
    reserved26 @26;
    reserved27 @27;
    reserved28 @28;
    reserved29 @29;
    reserved30 @30;

    promptSingleLow @31;
    promptSingleHigh @32;
  }
}

struct ModelManagerSP @0xaedffd8f31e7b55d {
  activeBundle @0 :ModelBundle;
  selectedBundle @1 :ModelBundle;
  availableBundles @2 :List(ModelBundle);

  struct DownloadUri {
    uri @0 :Text;
    sha256 @1 :Text;
  }

  enum DownloadStatus {
    notDownloading @0;
    downloading @1;
    downloaded @2;
    cached @3;
    failed @4;
    verifying @5;
  }

  struct DownloadProgress {
    status @0 :DownloadStatus;
    progress @1 :Float32;
    eta @2 :UInt32;
    speed @3 :Float32;
    canceling @4 :Bool;
    resuming @5 :Bool;
  }

  struct Chunk {
    fileName @0 :Text;
    sha256 @1 :Text;
  }

  struct Artifact {
    fileName @0 :Text;
    downloadUri @1 :DownloadUri;
    downloadProgress @2 :DownloadProgress;
    chunks @3 :List(Chunk);
  }

  struct Model {
    type @0 :Type;
    artifact @1 :Artifact;  # Main artifact
    metadata @2 :Artifact;  # Metadata artifact

    enum Type {
      supercombo @0;
      navigation @1;
      vision @2;
      policy @3;
      offPolicy @4;
      onPolicy @5;
      chunked @6;
    }
  }

  enum Runner {
    snpe @0;
    tinygrad @1;
    stock @2;
  }

  struct Override {
    key @0 :Text;
    value @1 :Text;
  }

  struct ModelBundle {
    index @0 :UInt32;
    internalName @1 :Text;
    displayName @2 :Text;
    models @3 :List(Model);
    status @4 :DownloadStatus;
    generation @5 :UInt32;
    environment @6 :Text;
    runner @7 :Runner;
    is20hz @8 :Bool;
    ref @9 :Text;
    minimumSelectorVersion @10 :UInt32;
    overrides @11 :List(Override);
  }
}

struct LongitudinalPlanSP @0xf35cc4560bbf6ec2 {
  longitudinalPlanSource @1 :LongitudinalPlanSource;
  smartCruiseControl @2 :SmartCruiseControl;
  speedLimit @3 :SpeedLimit;
  vTarget @4 :Float32;
  aTarget @5 :Float32;
  events @6 :List(OnroadEventSP.Event);
  e2eAlerts @7 :E2eAlerts;

  deprecated :group {
    dec @0 :DynamicExperimentalControlDEPRECATED;
  }

  struct DynamicExperimentalControlDEPRECATED {
    state @0 :DynamicExperimentalControlState;
    enabled @1 :Bool;
    active @2 :Bool;

    enum DynamicExperimentalControlState {
      acc @0;
      blended @1;
    }
  }

  struct SmartCruiseControl {
    vision @0 :Vision;
    map @1 :Map;

    struct Vision {
      state @0 :VisionState;
      vTarget @1 :Float32;
      aTarget @2 :Float32;
      currentLateralAccel @3 :Float32;
      maxPredictedLateralAccel @4 :Float32;
      enabled @5 :Bool;
      active @6 :Bool;
      curveDirection @7 :Int8; # turn being slowed for: 0 = none, -1 = left, 1 = right, 2 = S-bend
      # curve speed from the driver profile (CurveSpeedDriverProfile) only
      mapValid @8 :Bool; # mapd's curvature ahead is in use: the car is on its road, heading along it, with a recent GPS fix
      mapLimited @9 :Bool; # the map's curvature ahead, not the camera's, sets the braking now
      mapAccelRequired @10 :Float32; # m/s^2 the map's curvature ahead alone asks for (10 = nothing to slow for)
      visionAccelRequired @11 :Float32; # m/s^2 the camera's preview alone asks for (10 = nothing to slow for)
    }

    struct Map {
      state @0 :MapState;
      vTarget @1 :Float32;
      aTarget @2 :Float32;
      enabled @3 :Bool;
      active @4 :Bool;
    }

    enum VisionState {
      disabled @0; # System disabled or inactive.
      enabled @1; # No predicted substantial turn on vision range.
      entering @2; # A substantial turn is predicted ahead, adapting speed to turn comfort levels.
      turning @3; # Actively turning. Managing acceleration to provide a roll on turn feeling.
      leaving @4; # Road ahead straightens. Start to allow positive acceleration.
      overriding @5; # System overriding with manual control.
    }

    enum MapState {
      disabled @0; # System disabled or inactive.
      enabled @1; # No predicted substantial turn on map range.
      turning @2; # Actively turning. Managing acceleration to provide a roll on turn feeling.
      overriding @3; # System overriding with manual control.
    }
  }

  struct SpeedLimit {
    resolver @0 :Resolver;
    assist @1 :Assist;

    struct Resolver {
      speedLimit @0 :Float32;
      distToSpeedLimit @1 :Float32;
      source @2 :Source;
      speedLimitOffset @3 :Float32;
      speedLimitLast @4 :Float32;
      speedLimitFinal @5 :Float32;
      speedLimitFinalLast @6 :Float32;
      speedLimitValid @7 :Bool;
      speedLimitLastValid @8 :Bool;
      carSpeedLimit @9 :Float32;  # fork: m/s, the car's sign reading in use: a ramp's sign read from the mainline held back, 0 = none
    }

    struct Assist {
      state @0 :AssistState;
      enabled @1 :Bool;
      active @2 :Bool;
      vTarget @3 :Float32;
      aTarget @4 :Float32;
    }

    enum Source {
      none @0;
      car @1;
      map @2;
      baidu @3;   # Baidu Maps online data from roadeventd, replaces map (offline OSM) while fresh
    }

    enum AssistState {
      disabled @0;
      inactive @1; # No speed limit set or not enabled by parameter.
      preActive @2;
      pending @3; # Awaiting new speed limit.
      adapting @4; # Reducing speed to match new speed limit.
      active @5; # Cruising at speed limit.
    }
  }

  enum LongitudinalPlanSource {
    cruise @0;
    sccVision @1;
    sccMap @2;
    speedLimitAssist @3;
  }

  struct E2eAlerts {
    greenLightAlert @0 :Bool;
    leadDepartAlert @1 :Bool;
  }
}

struct OnroadEventSP @0xda96579883444c35 {
  events @0 :List(Event);

  struct Event {
    name @0 :EventName;

    # event types
    enable @1 :Bool;
    noEntry @2 :Bool;
    warning @3 :Bool;   # alerts presented only when  enabled or soft disabling
    userDisable @4 :Bool;
    softDisable @5 :Bool;
    immediateDisable @6 :Bool;
    preEnable @7 :Bool;
    permanent @8 :Bool; # alerts presented regardless of openpilot state
    overrideLateral @10 :Bool;
    overrideLongitudinal @9 :Bool;
  }

  enum EventName {
    lkasEnable @0;
    lkasDisable @1;
    manualSteeringRequired @2;
    manualLongitudinalRequired @3;
    silentLkasEnable @4;
    silentLkasDisable @5;
    silentBrakeHold @6;
    silentWrongGear @7;
    silentReverseGear @8;
    silentDoorOpen @9;
    silentSeatbeltNotLatched @10;
    silentParkBrake @11;
    controlsMismatchLateral @12;
    hyundaiRadarTracksConfirmed @13;
    experimentalModeSwitchedDEPRECATED @14;
    wrongCarModeAlertOnly @15;
    pedalPressedAlertOnly @16;
    laneTurnLeft @17;
    laneTurnRight @18;
    speedLimitPreActive @19;
    speedLimitActive @20;
    speedLimitChanged @21;
    speedLimitPending @22;
    e2eChime @23;
    laneChangeRoadEdge @24;
    bigModelReady @25;
  }
}

struct CarParamsSP @0x80ae746ee2596b11 {
  flags @0 :UInt32;        # flags for car specific quirks in sunnypilot
  safetyParam @1 : Int16;  # flags for sunnypilot's custom safety flags
  pcmCruiseSpeed @3 :Bool;
  intelligentCruiseButtonManagementAvailable @4 :Bool;
  enableGasInterceptor @5 :Bool;

  neuralNetworkLateralControl @2 :NeuralNetworkLateralControl;

  struct NeuralNetworkLateralControl {
    model @0 :Model;
    fuzzyFingerprint @1 :Bool;

    struct Model {
      path @0 :Text;
      name @1 :Text;
    }
  }
}

struct CarControlSP @0xa5cd762cd951a455 {
  mads @0 :ModularAssistiveDrivingSystem;
  params @1 :List(Param);
  leadOne @2 :LeadData;
  leadTwo @3 :LeadData;
  intelligentCruiseButtonManagement @4 :IntelligentCruiseButtonManagement;

  struct Param {
    key @0 :Text;
    type @2 :ParamType;
    value @3 :Data;

    valueDEPRECATED @1 :Text; # The data type change may cause issues with backwards compatibility.
  }

  enum ParamType {
    string @0;
    bool @1;
    int @2;
    float @3;
    time @4;
    json @5;
    bytes @6;
  }
}

struct BackupManagerSP @0xf98d843bfd7004a3 {
  backupStatus @0 :Status;
  restoreStatus @1 :Status;
  backupProgress @2 :Float32;
  restoreProgress @3 :Float32;
  lastError @4 :Text;
  currentBackup @5 :BackupInfo;
  backupHistory @6 :List(BackupInfo);

  enum Status {
    idle @0;
    inProgress @1;
    completed @2;
    failed @3;
  }

  struct Version {
    major @0 :UInt16;
    minor @1 :UInt16;
    patch @2 :UInt16;
    build @3 :UInt16;
    branch @4 :Text;
  }

  struct MetadataEntry {
    key @0 :Text;
    value @1 :Text;
    tags @2 :List(Text);
  }

  struct BackupInfo {
    deviceId @0 :Text;
    version @1 :UInt32;
    config @2 :Text;
    isEncrypted @3 :Bool;
    createdAt @4 :Text;  # ISO timestamp
    updatedAt @5 :Text;  # ISO timestamp
    sunnypilotVersion @6 :Version;
    backupMetadata @7 :List(MetadataEntry);
  }
}

struct CarStateSP @0xb86e6369214c01c8 {
  speedLimit @0 :Float32;
}

struct LiveMapDataSP @0xf416ec09499d9d19 {
  speedLimitValid @0 :Bool;
  speedLimit @1 :Float32;
  speedLimitAheadValid @2 :Bool;
  speedLimitAhead @3 :Float32;
  speedLimitAheadDistance @4 :Float32;
  roadName @5 :Text;
}

struct ModelDataV2SP @0xa1680744031fdb2d {
  laneTurnDirection @0 :TurnDirection;
  leftLaneChangeEdgeBlock @1 :Bool;
  rightLaneChangeEdgeBlock @2 :Bool;
  blendfulCurvature @3 :Float32;
  blendfulTrust @4 :Float32;
  modelCurvature @5 :Float32;
  blendfulGoingAround @6 :Bool;
  blendfulLeftProb @7 :Float32;
  blendfulRightProb @8 :Float32;
  blendfulLaneWidth @9 :Float32;

  enum TurnDirection {
    none @0;
    turnLeft @1;
    turnRight @2;
  }
}

# road prompts for the VW cluster (ACC_Events) from roadeventd: the model's slope ahead, the offline OSM map and Baidu Maps online
# data; also Baidu's speed limits for the speed limit resolver. The prompts are display only.
struct RoadEventsIC @0xcb9fd56c7057593a {
  event @0 :UInt8;            # ACC_Events value to show: 0 none, 4 speed limit ahead (a speed camera), 9 crossing, 10 roundabout,
                              # 12 traffic jam, 14 downhill, 15 uphill
  distance @1 :Float32;       # m from the car to the event, 0 = unknown
  source @2 :Source;
  grade @3 :Float32;          # current road grade, rise over run (liveLocationKalman pitch)
  gradeAhead @4 :Float32;     # steepest predicted grade ahead (modelV2 path in world frame), rise over run
  mapMatched @5 :Bool;        # the car is placed on an offline map road
  roadClass @6 :UInt8;        # its OSM highway class, as in mapd's offline tiles (1 motorway ... 13 living street)
  jamLength @7 :Float32;      # m, length of the jam ahead while event is 12
  baiduOnline @8 :Bool;       # a fresh Baidu cruise answer for the road ahead (asked less than baidu.STALE ago)
  speedLimit @9 :Float32;     # unused (always 0): Baidu sells its road info API, a road's speed limit, to companies only
  speedLimitValid @10 :Bool;  # unused (always false), see speedLimit
  speedLimitAhead @11 :Float32;          # m/s, limit of the next Baidu speed camera on the road ahead, 0 = none
  speedLimitAheadDistance @12 :Float32;  # m to it, negative once passed (reported until 50 m behind)
  jamFollow @13 :Bool;           # a traffic jam ahead asks for the relaxed following distance (Baidu, see prompts.JamAhead)
  jamFollowDistance @14 :Float32;  # m to the start of that jam
  promptAlways @15 :Bool;        # show the prompts in ACC standby and with the main switch off too (VwRoadEventPrompts 2)
  eventSpeedLimit @16 :Float32;  # m/s, the limit of a speed limit ahead prompt (event 4), 0 = none
  rampNear @17 :Bool;            # on a motorway or trunk, a link leaves or joins it within 300 m ahead or 200 m behind (OSM)

  enum Source {
    none @0;
    model @1;
    map @2;
    baidu @3;
  }
}

struct NavRouteIC @0xc2243c65e0340384 {
  # navd's Baidu route with its traffic, for roadeventd's traffic jams ahead (it plans no road ahead of its own meanwhile)
  requestTime @0 :UInt64;              # monotonic ns when the route was asked: its traffic is as old as that
  coordinates @1 :List(Coordinate);    # the route from the car to the destination, WGS-84, as navRoute
  jams @2 :List(Jam);                  # slow or worse runs of its traffic, by distance along the coordinates

  struct Coordinate {
    latitude @0 :Float64;
    longitude @1 :Float64;
  }

  struct Jam {
    start @0 :Float32;   # m along the route from its first coordinate
    length @1 :Float32;  # m
    status @2 :UInt8;    # Baidu's traffic status: 2 slow, 3 congested, 4 severely congested
  }
}

struct CustomReserved12 @0x9ccdc8676701b412 {
}

struct SelfdriveStateIC @0xcd96dafb67a082d0 {
  rainLevel @0 :UInt8;   # 0 dry, 1 rain, 2 heavy rain, held a while after it stops
  rainMode @1 :Bool;     # the rain following distance is selected
  jamMode @2 :Bool;      # the relaxed following distance is selected for a traffic jam ahead (selfdriveState.personality)
}

struct ControlsStateIC @0xb057204d7deadf3f {
  modelDesiredCurvature @0 :Float32;
}

struct LongitudinalPlanIC @0xbd443b539493bc68 {
  leadDistance @0 :Float32;
  vTarget @1 :Float32;
  # cruise coast-band gate: where the car's throttle-off / brake-on edges are believed to be
  coastBandHigh @2 :Float32;       # throttle-off edge, m/s^2
  coastBandLow @3 :Float32;        # brake-on edge, m/s^2
  coastBandOffset @4 :Float32;     # learnt throttle-off edge relative to the pitch coast estimate
  coastBandWidth @5 :Float32;      # learnt throttle-off minus brake-on
  coastBandLearnCount @6 :UInt32;  # car flag edges accepted this drive
  coastBandHolding @7 :Bool;       # a state change is being held back right now
  pullAwayAllowance @8 :Float32;   # accel a radar lead pulling away lets the command reach above the model, m/s^2
  e2eDeadbandMargin @9 :Float32;   # how far below the lead plan the model must ask before its request counts, m/s^2
}

struct CarStateIC @0xfc6241ed8877b611 {
  steeringCurvature @0 :Float32;
  steeringSlightlyPressed @1 :Bool;
  steerFaultWarning @2 :Bool;
  radarDisableFailed @3 :Bool;
  batteryDetails @4 :BatteryDetails;
  cruiseSpeedLimit @5 :Float32;
  cruiseSpeedLimitPredicative @6 :Float32;
  cruiseSpeedLimitPredicativeType @7 :UInt8;
  engineIdle @8 :Bool;
  brakeRequested @9 :Bool;
  driverDoorOpen @10 :Bool;
  rainAmount @11 :Float32;       # rain sensor amount, 0-100 %
  rainWiperRequest @12 :UInt8;   # wiper speed the rain sensor asks for, 0-7
  wiperSwitch @13 :UInt8;        # wiper stalk: 0 off, 1 interval/auto, 2 stage 1, 3 stage 2
  # the radar's own ACC on the gateway harness, which openpilot longitudinal replaces towards the car
  radarAccStatus @14 :UInt8;     # its ACC_18.ACC_Status_ACC: 2 standby, 3 regulating, 4 override, 5 shut-off, 6/7 fault
  radarAccFault @15 :Bool;       # its ACC has faulted, for the rest of the drive at 7

  struct BatteryDetails {
    capacity @0 :Float32;
    charge @1 :Float32;
    soc @2 :Float32;
    temperature @3 :Float32;
    heaterActive @4 :Bool;
    voltage @5 :Float32;
    current @6 :Float32;
    power @7 :Float32;
  }
}

struct CarControlIC @0xa30662f84033036c {
  curvatureControllerActive @0 :Bool;
  rollCompensation @1 :Float32;
  steerLimited @2 :Bool;
  forceRHDForBSM @3 :Bool;
  longComfortMode @4 :Bool;
  disableCarSteerAlerts @5 :Bool;
  cruiseSpeedLimit @6 :Bool;
  cruiseSpeedLimitPredicative @7 :Bool;
  cruiseSpeedLimitPredReactToSL @8 :Bool;
  cruiseSpeedLimitPredReactToCurves @9 :Bool;
  hudLeadFollowTime @10 :Float32;
  hudLeadDistance @11 :Float32;
  hudSpeedLimit @12 :Float32; # m/s, current speed limit resolved by sunnypilot (car camera or map), 0 = none
  hudSpeedLimitFromMap @13 :Bool;
  hudSpeedLimitAhead @14 :Bool;
  hudCurveSpeed @15 :Float32; # m/s, vision curve control target speed while slowing for a curve, 0 = none
  hudCurveDirection @16 :Int8; # direction of that curve: 0 = none, -1 = left, 1 = right, 2 = S-bend
  hudRoadEvent @17 :UInt8; # road prompt (ACC_Events value) from roadeventd: 0 = none, 4 speed limit ahead, 9 crossing, 10 roundabout,
                           # 12 traffic jam, 14 downhill, 15 uphill
  hudRoadEventDistance @18 :Float32; # m to that road event, 0 = unknown
  hudRoadEventAlways @19 :Bool; # show hudRoadEvent in ACC standby and with the main switch off too
  hudSpeedLimitOffer @20 :Float32; # m/s, the limit Speed Limit Assist is asking to set (preActive), 0 = none
  hudSpeedLimitOfferAhead @21 :Bool; # that limit lies ahead
  hudRoadEventSpeedLimit @22 :Float32; # m/s, the limit of a speed limit ahead road prompt (hudRoadEvent 4), 0 = none
  hudCarSpeedLimit @23 :Float32; # m/s, the car's sign reading as openpilot uses it (a ramp's sign read from the motorway held back), 0 = none
}

struct CarParamsIC @0xc86a3d38d13eb3ef {
  dashcamOnlyReason @0 :DashcamOnlyReason;

  enum DashcamOnlyReason {
    unknown @0;
    radarDisableEngineOn @1;
  }
}

struct LateralCurvatureParameters @0xa4f1eb3323f5f582 {
  liveValid @0 :Bool;
  version @1 :Int32;
  useParams @2 :Bool;
  currentCorrection @3 :Float32;
  currentBias @4 :Float32;
  currentBucketPoints @5 :UInt16;
  totalBucketPoints @6 :UInt16;
  calPerc @7 :Int8;
  bucketSpeed @8 :Int8;
  corrections @9 :List(Float32);
  counts @10 :List(UInt16);
  biases @11 :List(Float32);
  bucketCurvature @12 :Int8;
  fitValid @13 :List(Bool);
  previewCorrections @14 :List(Float32);
  previewValid @15 :List(Bool);
}
