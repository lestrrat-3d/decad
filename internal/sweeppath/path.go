package sweeppath

import "github.com/lestrrat-3d/decad/report"

// The path types are the public report package's, so the root package hands a
// caller's path to the sweep kernels unconverted.
type (
	// PairPath is a prescribed path for one body in a pair sweep.
	// Only PoseSegment and RigidDriftSegment implement it.
	PairPath = report.PairPath
	// PoseSegment joins two placements relative to the body's current placement.
	PoseSegment = report.PoseSegment
	// QuantityVec carries three components of one physical kind.
	QuantityVec = report.QuantityVec
	// RigidDriftSegment moves a center linearly while its orientation rotates.
	RigidDriftSegment = report.RigidDriftSegment
)
