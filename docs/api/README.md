# Generated API References

These references are generated from the service schemas in `services/`. They
describe the RPC surface that this repository currently builds:

- [Drawing](drawing.md)
- [InfernalRobotics](infernal-robotics.md)
- [KerbalAlarmClock](kerbal-alarm-clock.md)
- [LiDAR](li-dar.md)
- [RemoteTech](remote-tech.md)
- [SpaceCenter](space-center.md)
- [UI](ui.md)

`KRPC.Core.json` and `KRPC.DockingCamera.json` are intentionally absent. Each
uses the generator's unsupported `BYTES` type; Core is also unnecessary because
`krpc-mars` establishes its own RPC and streaming connections. The project
intentionally defers `BYTES` support: this retains all normal vessel and
gameplay automation while excluding only Core's advanced control-plane APIs and
the optional DockingCamera image data.
