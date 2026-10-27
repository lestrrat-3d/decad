package extent

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// ToFaceStop resolves a linear to-face stop at face into the signed stop
// coordinate along the sketch plane normal of frame, beside that coordinate's
// own proven axial displacement. The level is COMPUTED — from another body's
// face, in float — so the displacement is what keeps it from publishing
// itself as the level it denotes: it composes the exact distance from the
// held stop to the level the face's record denotes, offsetDelta (the
// offset's conversion), and the face's own axial displacement carried to the
// stop level.
//
// The face must be usable as a prism cap — the stop is the closed-form
// intersection of the sweep direction with the target surface, and the cap
// it emits is planar and perpendicular to the sweep — so the face must be an
// analytic Plane whose held normal is EXACTLY parallel to the sweep normal (a
// zero rational cross product, never a tolerance), which states no normal
// departure. A cap carries a CapRecord and axial displacement; a revolved
// straight wall instead carries a denoted meridian whose endpoints must all
// have one exact axial coordinate. Its radial basis must span an exact level
// set of the sweep (RevolvedPlaneStop). The stop is then one exact level c,
// and the cap charges no footprint-dependent departure. A non-planar face, a
// tilted or uncertain plane, or a face with neither certificate is refused
// (ErrUnsupported). A face coplanar with the sketch plane stops the sweep
// before it starts (ErrDegenerate).
//
// travel is 0 for a standalone ToFace — the target face supplies the sense:
// the sweep runs toward whichever side the face is on — and ±1 for a
// TwoSided side, whose face must lie on that side. The signed offset
// displaces the stop along the travel: positive overshoots the face,
// negative stops short of it. what names the caller in the messages.
func ToFaceStop(face *topo.Face, frame r3.Frame, travel, offset, offsetDelta float64,
	what string) (float64, float64, error) {
	pl, ok := face.Surface.(surfacegeom.Plane)
	if !ok {
		// The gate is an ANALYTIC planar surface (a Plane — an extrude/revolve
		// cap), not merely a flat face: a boolean-built face can be flat yet
		// carry a Faceted surface, and this evaluator cannot use it as a stop.
		// The message names the actual public surface type so the caller sees
		// why.
		return 0, 0, fmt.Errorf(`%w: ToFace requires a stop face with an analytic Plane surface whose normal is parallel to the sweep direction, not merely a flat face; this face's surface is %s, which this evaluator cannot use as a stop even when flat — choose any analytic planar face with that orientation, or use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, decaderr.ErrUnsupported, surfacegeom.SurfaceTypeName(face.Surface))
	}
	n := frame.N()
	if !ExactlyParallel(pl.Frame.N(), n) {
		return 0, 0, fmt.Errorf(`%w: ToFace requires the stop face's normal exactly parallel to the sweep direction; this face's normal is tilted from it, however slightly, and the perpendicular cap this evaluator builds would charge nothing for the tilt — choose a face exactly perpendicular to the sweep or use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, decaderr.ErrUnsupported)
	}
	if face.NormalBound != 0 {
		return 0, 0, fmt.Errorf(`%w: ToFace cannot use this face: its surface departs from its planar tag by a stated tilt, which a perpendicular cap does not charge; use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, decaderr.ErrUnsupported)
	}
	var c *big.Rat
	var k, axial float64
	if denoted := face.Denoted; denoted != nil {
		var ok bool
		c, ok = RevolvedPlaneStop(*denoted, frame)
		if !ok {
			return 0, 0, fmt.Errorf(`%w: ToFace cannot prove that this revolved face denotes one plane perpendicular to the sweep; use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, decaderr.ErrUnsupported)
		}
		k = 1
	} else {
		rec := face.CapRecord
		if rec == nil || !face.HasAxialDelta {
			return 0, 0, fmt.Errorf(`%w: ToFace cannot use this face: it states no record of the exact plane it denotes (a faceted boolean face, a placed copy, a sweep or patch cap); use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, decaderr.ErrUnsupported)
		}
		var ok bool
		c, k, ok = PlaneStop(rec.Frame, rec.Xform, rec.Z, frame)
		if !ok {
			return 0, 0, fmt.Errorf(`%w: ToFace requires the plane the stop face denotes to be exactly perpendicular to the sweep; on this sketch plane its level would vary across the footprint, which the perpendicular cap does not charge; use a Distance (or, inside a TwoSided extent, DistanceSide) extent`, decaderr.ErrUnsupported)
		}
		axial = face.AxialDelta
	}
	zFace := pl.Frame.Origin().Sub(frame.Origin()).Dot(n)
	tol := RelativeStopTolerance(math.Max(pl.Frame.Origin().Len(), frame.Origin().Len()))
	if math.Abs(zFace) <= tol {
		return 0, 0, fmt.Errorf(`%w: the stop face is coplanar with the sketch plane`, decaderr.ErrDegenerate)
	}
	if travel == 0 {
		travel = 1
		if zFace < 0 {
			travel = -1
		}
	} else if zFace*travel < 0 {
		return 0, 0, fmt.Errorf(`%w: the stop face lies on the other side of the sketch plane than %s sweeps`, decaderr.ErrDegenerate, what)
	}
	stop := zFace + travel*offset
	if stop*travel <= tol {
		return 0, 0, fmt.Errorf(`%w: the to-face offset pulls the stop behind the sketch plane`, decaderr.ErrDegenerate)
	}
	// The three terms displace the level along the same normal, so they sum:
	// the exact distance from the held stop to the level the record denotes
	// (it covers the rounded tag origin, the difference, the dot product and
	// the offset step alike), the offset's conversion into millimetres, and
	// the face's proven axial displacement carried to a level displacement.
	if k != 1 {
		axial = proofbound.ProductUpper(k, axial)
	}
	return stop, proofbound.AbsSumUpper(StopLevelError(c, travel, offset, stop), offsetDelta, axial), nil
}

// ToFaceDenotedLevel is the exact level a to-face stop denotes, for a stop
// ToFaceStop resolved at face to the held level stop. It is the face's cap
// record at its denoted level, carried onto the sweep frame exactly
// (PlaneStopAt), plus the exact offset along the travel the stop's sign
// gives. It is nil when the face is no cap record with a denoted level, or
// offsetExact is nil. The caller records it through
// prismplacement.DenoteLevel, which refuses it unless the held stop lies
// within its displacement.
func ToFaceDenotedLevel(face *topo.Face, frame r3.Frame, stop float64, offsetExact *big.Rat) *big.Rat {
	rec := face.CapRecord
	if face.Denoted != nil || rec == nil || rec.ZDenoted == nil || !face.HasAxialDelta || offsetExact == nil {
		return nil
	}
	c, _, ok := PlaneStopAt(rec.Frame, rec.Xform, rec.ZDenoted, frame)
	if !ok {
		return nil
	}
	if stop < 0 {
		return c.Sub(c, offsetExact)
	}
	return c.Add(c, offsetExact)
}
