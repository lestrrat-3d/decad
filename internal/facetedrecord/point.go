package facetedrecord

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// PointSectionRecord keeps the authenticated Sketch geometry beside the
// faceted boundary of a point-section loft. The far section is never replaced
// by a fitted or scaled profile.
type PointSectionRecord struct {
	Apex                r3.Vec
	Profile             momentinput.Profile
	Plane               sectionrecord.PlaneRecord
	Source              *PointSectionSource
	SourceBox           [4]float64             // U min/max, V min/max; contains the true region and station polygon
	SourceBoxIssued     [4]float64             // original source rectangle at LoftFromPoint publication
	SourceAreaDelta     float64                // area of the symmetric difference between source region and station polygon
	SourceStation       []sectionrecord.Point2 // original local station coordinates, in source loop order
	SourceStationIssued []sectionrecord.Point2 // station coordinates published with the original point loft
	SourceStationRound  float64                // local error of every held station against its exact curve point
	SourceRoundIssued   float64                // station error published with the original point loft
	SourceArcUpper      []float64              // per-cell source curve speed in world millimetres
	SourceMatched       []float64              // per-cell parameter-matched source departure in world millimetres
}

// PointSectionSource names the live Sketch entities authenticated when the
// point loft was made. A later structural join can request a new certificate
// only from these entities and must refuse if their revision or frame moved.
type PointSectionSource struct {
	Sketch   *sketch.Sketch
	Profile  *sketch.Profile
	Revision uint64
	Frame    r3.Frame
	Entities []sketch.Entity
}

// A point cone trim uses the original point-loft fan as its radial domain.
// Chained trims always start from that same authenticated fan rather than a
// previously rounded Boolean mesh.
type PointConeTrimRecord struct {
	Base  Payload
	Lower *PointConeLimit
	Upper *PointConeLimit
}

type PointConeLimit struct {
	Apex, Far, Slope *big.Rat
	Surface          surfacegeom.Cone
	Origins          []provenance.Ref
	Denoted          *surfacenormal.Revolved
	Reversed         bool

	Consts *pointcone.Consts // built on first use by Constants
}

func (cone *PointConeLimit) Constants() *pointcone.Consts {
	if cone.Consts == nil {
		cone.Consts = pointcone.ConstantsOf(cone.Apex, cone.Slope)
	}
	return cone.Consts
}

// PointPlaneSimilarityRecord retains the exact source cut maps and the order
// of their later rigid and positive uniform images. The source slice is never
// reconstructed from the rounded cap chords.
type PointPlaneSimilarityRecord struct {
	Slice *PointPlaneSliceRecord
	Steps []PointPlaneSimilarityStep
}

type PointPlaneSimilarityStep struct {
	Motion          r3.Transform
	Anchor          r3.Vec
	Factor          float64 // zero means a rigid step
	MeshBound       float64
	CapDisplacement float64
}

// PointPlaneSliceRecord always names the original point loft. Repeated cuts
// never use a previously rounded cut ring as their geometric source.
type PointPlaneSliceRecord struct {
	Base         Payload
	Lower, Upper *PointPlaneLimit
}

// PointPlaneLimit is the exact plane through the source line of an unplaced
// world-XY chain Extrude. Its normal is deliberately unnormalised: all tests
// and ray intersections below are rational statements about stored floats.
type PointPlaneLimit struct {
	Nx, Ny, D               *big.Rat
	A, B                    sectionrecord.Point2
	Z0, Z1                  float64
	Surface                 surfacegeom.Plane
	Origins                 []provenance.Ref
	NormalBound, MapStretch float64
	Centroid                measurement.VecMeasurement
	HasCentroid             bool
	// Whether the sheet's reported normal faces away from the apex.
	FrameTowardFar bool
}

func ratOfPointPlane(v float64) *big.Rat { return proofarith.FloatRat(v) }

func pointPlaneAbs(q *big.Rat) *big.Rat { return new(big.Rat).Abs(q) }

// pointPlaneAdd, pointPlaneSub and pointPlaneMul return a fresh Rat and read
// their operands only. Operands must be in lowest terms: the result always
// equals the exact value, but is in lowest terms only for reduced operands (a
// 3/9 operand gives 3/9 where big.Rat gives 1/3). Every producer in this
// package reduces: FloatRat through ratOfPointPlane and lfF, big.NewRat
// literals, lfOne, lfZero, and results of these helpers and of raw Quo, Abs and
// Neg. The guard TestNoRatWritesThroughNumDenom (internal/proof) bars in-place
// Num and Denom writes outside internal/proof.
func pointPlaneAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }

func pointPlaneSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }

func pointPlaneMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }

func (p *PointPlaneLimit) NormalL1() *big.Rat {
	return pointPlaneAdd(pointPlaneAbs(p.Nx), pointPlaneAbs(p.Ny))
}

func (p *PointPlaneLimit) Denominator(apex, far r3.Vec) *big.Rat {
	x := pointPlaneSub(ratOfPointPlane(far.X), ratOfPointPlane(apex.X))
	y := pointPlaneSub(ratOfPointPlane(far.Y), ratOfPointPlane(apex.Y))
	return pointPlaneAdd(pointPlaneMul(p.Nx, x), pointPlaneMul(p.Ny, y))
}
