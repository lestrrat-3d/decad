package report

import (
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// PairPath names one body's motion during a two-body sweep. Affine source-box,
// source-sphere, and certified faceted-floor paths, co-translating oblique
// source boxes, rotating source-box or centered-sphere rigid drifts, and any
// path of two exact planar solids can receive continuous certificates; other
// valid paths report an undecided sweep.
type PairPath interface{ pairPath() }

// PoseSegment joins two placements relative to the body's current placement.
type PoseSegment struct {
	From, To r3.Transform
	Duration units.Value
}

func (PoseSegment) pairPath() {}

// QuantityVec carries three components of one physical kind.
type QuantityVec struct{ X, Y, Z units.Value }

// RigidDriftSegment moves a center linearly while its orientation rotates.
type RigidDriftSegment struct {
	From            r3.Transform
	Center          r3.Vec
	LinearVelocity  QuantityVec
	AngularVelocity QuantityVec
	Duration        units.Value
}

func (RigidDriftSegment) pairPath() {}
