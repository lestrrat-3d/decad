package reportvocab

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/units"
)

// LinkageReport records VerifyLinkage's drive, pair results, and verdict.
// Each type parameter carries the caller's own model identity.
type LinkageReport[BodyT comparable, LinkageT, LinkT, DriveT, PoseT, CellT any] struct {
	Request           MotionRequest
	ReadingResolution units.Value
	Linkage           LinkageT
	Drive             DriveT
	Links             []LinkT
	JointContacts     []DiagnosticPair[BodyT]
	Against           []BodyT
	Poses             []LinkagePoseResult[BodyT, PoseT, CellT]
	Intervals         []MotionInterval
	Collisions        []LinkCollision[BodyT]
	Clearance         *ScalarReading
	Assessment        Assessment
	Diagnostics       []Diagnostic[BodyT, CellT]
	Status            Status
}

// Passed reports whether the report is Sound. It returns false for nil.
func (r *LinkageReport[BodyT, LinkageT, LinkT, DriveT, PoseT, CellT]) Passed() bool {
	return r != nil && r.Status == Sound
}

// LinkagePoseResult records one evaluated linkage pose and its pair findings.
type LinkagePoseResult[BodyT comparable, PoseT, CellT any] struct {
	Pose          PoseT
	Interferences []Interference[BodyT]
	Clearances    []Clearance[BodyT]
	Diagnostics   []Diagnostic[BodyT, CellT]
}

// LinkCollision is a proven overlap at one ideal linkage pose.
type LinkCollision[BodyT comparable] struct {
	At     units.Value
	A, B   BodyT
	Volume measurement.Measurement
}
