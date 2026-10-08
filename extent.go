package decad

import "github.com/lestrrat-3d/decad/internal/extent"

// Direction is the sense of a standalone one-sided extent.
type Direction = extent.Direction

const (
	Along   = extent.Along
	Against = extent.Against
)

// Extent is the sealed set of linear extents accepted by Extrude.
type Extent = extent.Extent

// SideExtent is one side of a TwoSided extent.
type SideExtent = extent.SideExtent

// Distance sweeps a non-negative distance in Direction Dir.
type Distance = extent.Distance

// ThroughAll sweeps to the farthest live body in Direction Dir.
type ThroughAll = extent.ThroughAll

// Symmetric sweeps both ways from the sketch plane.
type Symmetric = extent.Symmetric

// TwoSided sweeps each side independently.
type TwoSided = extent.TwoSided

// DistanceSide states a distance for one side of a TwoSided extent.
type DistanceSide = extent.DistanceSide

// ThroughAllSide sweeps one side to the farthest live body.
type ThroughAllSide = extent.ThroughAllSide

// ToFace stops at a selected face of Body, displaced by Offset.
type ToFace = extent.ToFace[*Body, FaceSelector]

// AngularExtent is the sealed set of angular extents accepted by Revolve.
type AngularExtent = extent.AngularExtent

// SideAngular is one side of a TwoSidedAngle extent.
type SideAngular = extent.SideAngular

// AngleExtent sweeps a non-negative angle in Direction Dir.
type AngleExtent = extent.AngleExtent

// FullRevolution sweeps one full turn.
type FullRevolution = extent.FullRevolution

// SymmetricAngle sweeps both angular ways from the sketch plane.
type SymmetricAngle = extent.SymmetricAngle

// TwoSidedAngle sweeps each angular side independently.
type TwoSidedAngle = extent.TwoSidedAngle

// AngleSide states an angle for one side of a TwoSidedAngle extent.
type AngleSide = extent.AngleSide

// ToFaceAngular stops at a selected face of Body.
type ToFaceAngular = extent.ToFaceAngular[*Body, FaceSelector]

func normalizeExtent(e Extent) (Extent, error) {
	return extent.NormalizeExtent[*Body, FaceSelector](e)
}

func normalizeSideExtent(s SideExtent) (SideExtent, error) {
	return extent.NormalizeSideExtent[*Body, FaceSelector](s)
}

func normalizeAngularExtent(a AngularExtent) (AngularExtent, error) {
	return extent.NormalizeAngularExtent[*Body, FaceSelector](a)
}

func normalizeSideAngular(s SideAngular) (SideAngular, error) {
	return extent.NormalizeSideAngular[*Body, FaceSelector](s)
}
