# Layout of mapd's offline map tiles (/data/media/0/osm/offline), copied from pfeiferj/mapd cereal/offline/offline.capnp
# (MIT, 2026-07-16: highwayClass and id added). Only the binary layout matters for reading: keep field order and types.
@0xda3a0d9284ca402f;

enum HighwayClass {
  unknown @0;
  motorway @1;
  motorwayLink @2;
  trunk @3;
  trunkLink @4;
  primary @5;
  primaryLink @6;
  secondary @7;
  secondaryLink @8;
  tertiary @9;
  tertiaryLink @10;
  unclassified @11;
  residential @12;
  livingStreet @13;
}

struct Way {
  name @0 :Text;
  ref @1 :Text;
  maxSpeed @2 :Float64;
  minLat @3 :Float64;
  minLon @4 :Float64;
  maxLat @5 :Float64;
  maxLon @6 :Float64;
  nodes @7 :List(Coordinates);
  lanes @8 :UInt8;
  advisorySpeed @9 :Float64;
  hazard @10 :Text;
  oneWay @11 :Bool;   # also set by mapd for junction=roundabout and motorways without an oneway tag
  maxSpeedForward @12 :Float64;
  maxSpeedBackward @13 :Float64;
  id @14 :Int64;
  highwayClass @15 :HighwayClass;
  maxSpeedConditional @16 :Text;
  maxSpeedForwardConditional @17 :Text;
  maxSpeedBackwardConditional @18 :Text;
}

struct Coordinates {
  latitude @0 :Float64;
  longitude @1 :Float64;
}

struct Offline {
  minLat @0 :Float64;
  minLon @1 :Float64;
  maxLat @2 :Float64;
  maxLon @3 :Float64;
  ways @4 :List(Way);
  overlap @5 :Float64;
}
