package selectorquery

import (
	"fmt"
	"math"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const (
	ConvexKind        = "convex"
	ConcaveKind       = "concave"
	ParallelToKind    = "parallel_to"
	EndpointAtKind    = "endpoint_at"
	LongerThanKind    = "longer_than"
	CreatedByKind     = "created_by"
	OuterLoopOfKind   = "outer_loop_of"
	CircularKind      = "circular"
	PlanarKind        = "planar"
	CylindricalKind   = "cylindrical"
	NormalToKind      = "normal_to"
	FacingKind        = "facing"
	FaceCreatedByKind = "face_created_by"
	WallsKind         = "walls"
	FreeKind          = "free"
)

type EdgeClause[R comparable] struct {
	Kind      string
	Direction r3.Vec
	Point     r3.Vec
	Length    units.Value
	Ref       R
}

type FaceClause[R comparable] struct {
	Kind      string
	Direction r3.Vec
	Ref       R
}

type EdgeView[R comparable] interface {
	SelectorConvex() bool
	SelectorFree() bool
	SelectorLineDirection() (r3.Vec, bool)
	SelectorEndpoints() (r3.Vec, r3.Vec)
	SelectorLengthMM() float64
	SelectorHasOrigin(R) bool
	SelectorOnOuterLoop(R) bool
	SelectorCircular() bool
}

type FaceView[R comparable] interface {
	SelectorPlanarNormal() (r3.Vec, bool)
	SelectorOutwardPlanarNormal() (r3.Vec, bool)
	SelectorCylindrical() bool
	SelectorHasOrigin(R) bool
	// SelectorIsWallOf reports whether the face carries a side(i, j) role of
	// the producer R names.
	SelectorIsWallOf(R) bool
}

// Validate rejects a malformed clause before looking at any edge.
func (p EdgeClause[R]) Validate(validateRef func(R, string) error) error {
	switch p.Kind {
	case ConvexKind, ConcaveKind, CircularKind, FreeKind:
		return nil
	case CreatedByKind:
		return validateRef(p.Ref, "created-by")
	case OuterLoopOfKind:
		return validateRef(p.Ref, "outer-loop-of")
	case ParallelToKind:
		return ValidateDirection(p.Direction, "parallel-to")
	case EndpointAtKind:
		for _, c := range []float64{p.Point.X, p.Point.Y, p.Point.Z} {
			if math.IsNaN(c) || math.IsInf(c, 0) {
				return fmt.Errorf(`%w: an endpoint-at position component is not finite`, decaderr.ErrNotFinite)
			}
		}
		return nil
	case LongerThanKind:
		_, err := sectionrecord.MagnitudeIn(p.Length, units.Length, units.Millimeter, "the longer-than length")
		return err
	case "":
		return fmt.Errorf(`%w: edge predicate names no kind; use the package constructors`, decaderr.ErrDegenerate)
	default:
		return fmt.Errorf(`%w: unknown edge predicate kind %q`, decaderr.ErrDegenerate, p.Kind)
	}
}

// Validate rejects a malformed clause before looking at any face.
func (p FaceClause[R]) Validate(validateRef func(R, string) error) error {
	switch p.Kind {
	case PlanarKind, CylindricalKind:
		return nil
	case FaceCreatedByKind:
		return validateRef(p.Ref, "face-created-by")
	case WallsKind:
		return validateRef(p.Ref, "walls")
	case NormalToKind:
		return ValidateDirection(p.Direction, "normal-to")
	case FacingKind:
		return ValidateDirection(p.Direction, "facing")
	case "":
		return fmt.Errorf(`%w: face predicate names no kind; use the package constructors`, decaderr.ErrDegenerate)
	default:
		return fmt.Errorf(`%w: unknown face predicate kind %q`, decaderr.ErrDegenerate, p.Kind)
	}
}

// Matches checks one edge clause against the live topology supplied by root.
func (p EdgeClause[R]) Matches(e EdgeView[R]) bool {
	switch p.Kind {
	case ConvexKind:
		return e.SelectorConvex()
	case ConcaveKind:
		return !e.SelectorConvex()
	case FreeKind:
		return e.SelectorFree()
	case ParallelToKind:
		d, ok := e.SelectorLineDirection()
		return ok && ParallelDirs(d, p.Direction)
	case EndpointAtKind:
		start, end := e.SelectorEndpoints()
		return start == p.Point || end == p.Point
	case LongerThanKind:
		mm, err := p.Length.In(units.Millimeter)
		return err == nil && e.SelectorLengthMM() > mm
	case CreatedByKind:
		return e.SelectorHasOrigin(p.Ref)
	case OuterLoopOfKind:
		return e.SelectorOnOuterLoop(p.Ref)
	case CircularKind:
		return e.SelectorCircular()
	default:
		return false
	}
}

// Matches checks one face clause against the live topology supplied by root.
func (p FaceClause[R]) Matches(f FaceView[R]) bool {
	switch p.Kind {
	case PlanarKind:
		_, ok := f.SelectorPlanarNormal()
		return ok
	case CylindricalKind:
		return f.SelectorCylindrical()
	case NormalToKind:
		n, ok := f.SelectorPlanarNormal()
		return ok && ParallelDirs(n, p.Direction)
	case FacingKind:
		n, ok := f.SelectorOutwardPlanarNormal()
		if !ok {
			return false
		}
		d := ScaleToUnitInfNorm(p.Direction)
		return ParallelDirs(n, d) && n.Dot(d) > 0
	case FaceCreatedByKind:
		return f.SelectorHasOrigin(p.Ref)
	case WallsKind:
		return f.SelectorIsWallOf(p.Ref)
	default:
		return false
	}
}

func (p EdgeClause[R]) Render(renderRef func(R) string) string {
	switch p.Kind {
	case ConvexKind, ConcaveKind, CircularKind, FreeKind:
		return p.Kind
	case ParallelToKind:
		return p.Kind + "(" + RenderVec(p.Direction) + ")"
	case EndpointAtKind:
		return p.Kind + "(" + RenderVec(p.Point) + ")"
	case LongerThanKind:
		return p.Kind + "(" + p.Length.String() + ")"
	case CreatedByKind, OuterLoopOfKind:
		return p.Kind + "(" + renderRef(p.Ref) + ")"
	default:
		return "<invalid>"
	}
}

func (p FaceClause[R]) Render(renderRef func(R) string) string {
	switch p.Kind {
	case PlanarKind, CylindricalKind:
		return p.Kind
	case NormalToKind, FacingKind:
		return p.Kind + "(" + RenderVec(p.Direction) + ")"
	case FaceCreatedByKind, WallsKind:
		return p.Kind + "(" + renderRef(p.Ref) + ")"
	default:
		return "<invalid>"
	}
}

const parallelEps = 1e-9

// ParallelDirs checks either sense and rescales only when ordinary lengths overflow or underflow.
func ParallelDirs(a, b r3.Vec) bool {
	if a == (r3.Vec{}) || b == (r3.Vec{}) {
		return false
	}
	la, lb := a.Len(), b.Len()
	cl := a.Cross(b).Len()
	prod := parallelEps * la * lb
	if !math.IsInf(la, 0) && !math.IsInf(lb, 0) && !math.IsInf(cl, 0) &&
		!math.IsInf(prod, 0) && prod > 0 {
		return cl <= prod
	}
	a, b = ScaleToUnitInfNorm(a), ScaleToUnitInfNorm(b)
	return a.Cross(b).Len() <= parallelEps*a.Len()*b.Len()
}

// ScaleToUnitInfNorm keeps an extreme finite direction on the same ray.
func ScaleToUnitInfNorm(v r3.Vec) r3.Vec {
	m := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
	return r3.NewVec(v.X/m, v.Y/m, v.Z/m)
}

// ValidateDirection rejects a zero or non-finite predicate direction.
func ValidateDirection(v r3.Vec, what string) error {
	for _, c := range []float64{v.X, v.Y, v.Z} {
		if math.IsNaN(c) || math.IsInf(c, 0) {
			return fmt.Errorf(`%w: a %s direction component is not finite`, decaderr.ErrNotFinite, what)
		}
	}
	if v == (r3.Vec{}) {
		return fmt.Errorf(`%w: a zero %s direction names no direction`, decaderr.ErrDegenerate, what)
	}
	return nil
}

// RenderVec uses shortest round-tripping coordinates and normalizes negative zero.
func RenderVec(v r3.Vec) string {
	return RenderCoord(v.X) + "," + RenderCoord(v.Y) + "," + RenderCoord(v.Z)
}

func RenderCoord(c float64) string {
	if c == 0 {
		return "0"
	}
	return strconv.FormatFloat(c, 'g', -1, 64)
}
