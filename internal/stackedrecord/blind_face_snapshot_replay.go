package stackedrecord

import (
	"context"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type blindFaceSnapshot struct {
	face     *topo.Face
	origin   provenance.Ref
	scalars  [6]uint64
	flags    [5]bool
	centroid measurement.VecMeasurement
	surface  surfacegeom.Surface
	cap      *topo.CapRecord
	capValue topo.CapRecord
	capRat   *big.Rat
	capCopy  *big.Rat
	loops    []blindLoopSnapshot
}
type blindLoopSnapshot struct {
	loop  *topo.Loop
	outer bool
	edges []blindEdgeSnapshot
}
type blindEdgeSnapshot struct {
	edge                 *topo.Edge
	forward              bool
	start, end           *topo.Vertex
	startPos, endPos     r3.Vec
	startBound, endBound units.Value
	curve                surfacegeom.Curve
	faces                [2]*topo.Face
	count                int
	scalars              [4]uint64
	flags                [4]bool
	selector             r3.Vec
}

func blindFaceBits(f *topo.Face) ([6]uint64, [5]bool) {
	return [6]uint64{math.Float64bits(f.Area), math.Float64bits(f.AreaBound), math.Float64bits(f.AxialDelta), math.Float64bits(f.NormalBound), math.Float64bits(f.HeldNormalBound), math.Float64bits(f.TagDeparture)}, [5]bool{f.HasCentroid, f.HasAxialDelta, f.HeldPlanar, f.Reversed, f.RetainedTag}
}
func blindEdgeBits(e *topo.Edge) ([4]uint64, [4]bool) {
	return [4]uint64{math.Float64bits(e.Length), math.Float64bits(e.LengthBound), math.Float64bits(e.CurveBound), math.Float64bits(e.ChainBound)}, [4]bool{e.SelectorDirectionProven, e.Convex, e.LengthUnbounded, e.CurveBounded}
}
func blindUnitBits(a, b units.Value) bool {
	if len(a.Unit().Symbol()) > 64 || len(b.Unit().Symbol()) > 64 {
		return false
	}
	return a.Unit() == b.Unit() && math.Float64bits(a.Mag()) == math.Float64bits(b.Mag())
}
func blindSurfaceEqual(a, b surfacegeom.Surface) bool {
	switch x := a.(type) {
	case surfacegeom.Plane:
		y, ok := b.(surfacegeom.Plane)
		return ok && replaySameFrameBits(x.Frame, y.Frame)
	case surfacegeom.Cylinder:
		y, ok := b.(surfacegeom.Cylinder)
		return ok && replaySameVecBits(x.Origin, y.Origin) && replaySameVecBits(x.Axis, y.Axis) && blindUnitBits(x.Radius, y.Radius)
	default:
		return false
	}
}
func blindCurveEqual(a, b surfacegeom.Curve) bool {
	switch x := a.(type) {
	case surfacegeom.Line3:
		_, ok := b.(surfacegeom.Line3)
		return ok
	case surfacegeom.Circle3:
		y, ok := b.(surfacegeom.Circle3)
		return ok && replaySameVecBits(x.Center, y.Center) && replaySameVecBits(x.Axis, y.Axis) && blindUnitBits(x.Radius, y.Radius)
	case surfacegeom.Arc3:
		y, ok := b.(surfacegeom.Arc3)
		return ok && replaySameVecBits(x.Center, y.Center) && replaySameVecBits(x.Axis, y.Axis) && blindUnitBits(x.Radius, y.Radius)
	default:
		return false
	}
}
func blindCentroidEqual(a, b measurement.VecMeasurement) bool {
	return a.Exactness == b.Exactness && replaySameVecBits(a.Value, b.Value) && blindUnitBits(a.Bound, b.Bound)
}

func snapshotBlindFaces(ctx context.Context, events []ReplayBlindFaceEvent, b *proofbound.ReplayBudget) ([]blindFaceSnapshot, bool, error) {
	count, edges := 0, 0
	for _, ev := range events {
		if err := b.SpendWork(ctx, 32); err != nil {
			return nil, false, err
		}
		f := ev.Face
		if f == nil || f.Denoted != nil || f.Source != nil || len(f.Origins) != 1 || len(f.Origins[0].Role) > 64 || len(f.Upstream) != 0 || !blindSurfaceEqual(f.Surface, f.Surface) || len(f.Loops) > 4096-count {
			return nil, false, nil
		}
		count += len(f.Loops)
		for _, l := range f.Loops {
			if err := b.SpendWork(ctx, 32); err != nil {
				return nil, false, err
			}
			if l == nil || len(l.CoEdges) > 4096-edges {
				return nil, false, nil
			}
			edges += len(l.CoEdges)
		}
	}
	if err := b.ReserveRecords(ctx, uint64(len(events)+count+edges)); err != nil {
		return nil, false, err
	}
	for _, a := range []struct{ n, size uint64 }{{uint64(len(events)), uint64(unsafe.Sizeof(blindFaceSnapshot{}))}, {uint64(count), uint64(unsafe.Sizeof(blindLoopSnapshot{}))}, {uint64(edges), uint64(unsafe.Sizeof(blindEdgeSnapshot{}))}} {
		if err := b.ReserveArray(ctx, a.n, a.size); err != nil {
			return nil, false, err
		}
	}
	faces := make([]blindFaceSnapshot, len(events))
	loops := make([]blindLoopSnapshot, count)
	es := make([]blindEdgeSnapshot, edges)
	li, ei := 0, 0
	for i, ev := range events {
		f := ev.Face
		if err := b.SpendWork(ctx, 128+uint64(len(f.Origins[0].Role))); err != nil {
			return nil, false, err
		}
		s := blindFaceSnapshot{face: f, origin: f.Origins[0], centroid: f.Centroid, surface: f.Surface, cap: f.CapRecord}
		s.scalars, s.flags = blindFaceBits(f)
		if f.CapRecord != nil {
			s.capValue = *f.CapRecord
			s.capRat = f.CapRecord.ZDenoted
			if s.capRat != nil {
				n, d := s.capRat.Num().BitLen(), s.capRat.Denom().BitLen()
				if n > 4096 || d > 4096 || s.capRat.Denom().Sign() <= 0 {
					return nil, false, nil
				}
				words := uint64((n+63)/64 + (d+63)/64 + 2)
				if err := b.SpendWork(ctx, words*16); err != nil {
					return nil, false, err
				}
				if err := b.ReserveRecords(ctx, 3); err != nil {
					return nil, false, err
				}
				if err := b.ReserveBytes(ctx, words*32+512); err != nil {
					return nil, false, err
				}
				s.capCopy = new(big.Rat).Set(s.capRat)
			}
		}
		s.loops = loops[li : li+len(f.Loops) : li+len(f.Loops)]
		li += len(f.Loops)
		for j, l := range f.Loops {
			ls := blindLoopSnapshot{loop: l, outer: l.Outer, edges: es[ei : ei+len(l.CoEdges) : ei+len(l.CoEdges)]}
			ei += len(l.CoEdges)
			for k, ce := range l.CoEdges {
				if err := b.SpendWork(ctx, 128); err != nil {
					return nil, false, err
				}
				e := ce.Edge
				if e == nil || e.Start == nil || e.End == nil || len(e.Faces) != 2 || e.RetainedCurve || e.Level != (denotation.LevelToken{}) || e.Denot != (denotation.CurveToken{}) || e.Start.Level != (denotation.LevelToken{}) || e.End.Level != (denotation.LevelToken{}) || e.Start.Denot != (denotation.CurveToken{}) || e.End.Denot != (denotation.CurveToken{}) || !blindCurveEqual(e.Curve, e.Curve) {
					return nil, false, nil
				}
				v := blindEdgeSnapshot{edge: e, forward: ce.Forward, start: e.Start, end: e.End, startPos: e.Start.Position, endPos: e.End.Position, startBound: e.Start.Bound, endBound: e.End.Bound, curve: e.Curve, count: len(e.Faces), selector: e.SelectorDirection}
				v.faces[0], v.faces[1] = e.Faces[0], e.Faces[1]
				v.scalars, v.flags = blindEdgeBits(e)
				ls.edges[k] = v
			}
			s.loops[j] = ls
		}
		faces[i] = s
	}
	return faces, true, nil
}

func recheckBlindFaces(ctx context.Context, faces []blindFaceSnapshot, b *proofbound.ReplayBudget) (bool, error) {
	for _, s := range faces {
		if err := b.SpendWork(ctx, 128+uint64(len(s.origin.Role))); err != nil {
			return false, err
		}
		f := s.face
		if f == nil || f.Denoted != nil || f.Source != nil || len(f.Origins) != 1 || f.Origins[0] != s.origin || len(f.Upstream) != 0 || f.CapRecord != s.cap || len(f.Loops) != len(s.loops) || !blindSurfaceEqual(f.Surface, s.surface) || !blindCentroidEqual(f.Centroid, s.centroid) {
			return false, nil
		}
		bits, flags := blindFaceBits(f)
		if bits != s.scalars || flags != s.flags {
			return false, nil
		}
		if s.cap != nil {
			c := f.CapRecord
			if !replaySameFrameBits(c.Frame, s.capValue.Frame) || !replaySameTransformBits(c.Xform, s.capValue.Xform) || math.Float64bits(c.Z) != math.Float64bits(s.capValue.Z) || c.ZDenoted != s.capRat {
				return false, nil
			}
			if s.capRat != nil {
				if s.capRat.Num().BitLen() > 4096 || s.capRat.Denom().BitLen() > 4096 || s.capRat.Denom().Sign() <= 0 {
					return false, nil
				}
				cmp, err := proof.ReplayRatCompare(ctx, s.capRat, s.capCopy, b)
				if err != nil || cmp != 0 {
					return false, err
				}
			}
		}
		for i, ls := range s.loops {
			if err := b.SpendWork(ctx, 32); err != nil {
				return false, err
			}
			l := f.Loops[i]
			if l == nil || l != ls.loop || l.Outer != ls.outer || len(l.CoEdges) != len(ls.edges) {
				return false, nil
			}
			for j, old := range ls.edges {
				if err := b.SpendWork(ctx, 128); err != nil {
					return false, err
				}
				ce := l.CoEdges[j]
				e := ce.Edge
				if e == nil || e != old.edge || ce.Forward != old.forward || e.Start != old.start || e.End != old.end || e.Start == nil || e.End == nil || len(e.Faces) != old.count || !blindCurveEqual(e.Curve, old.curve) || !replaySameVecBits(e.SelectorDirection, old.selector) || !replaySameVecBits(e.Start.Position, old.startPos) || !replaySameVecBits(e.End.Position, old.endPos) || !blindUnitBits(e.Start.Bound, old.startBound) || !blindUnitBits(e.End.Bound, old.endBound) || e.Level != (denotation.LevelToken{}) || e.Denot != (denotation.CurveToken{}) || e.Start.Level != (denotation.LevelToken{}) || e.End.Level != (denotation.LevelToken{}) || e.Start.Denot != (denotation.CurveToken{}) || e.End.Denot != (denotation.CurveToken{}) {
					return false, nil
				}
				scalars, flags := blindEdgeBits(e)
				if scalars != old.scalars || flags != old.flags || e.Faces[0] != old.faces[0] || e.Faces[1] != old.faces[1] || e.RetainedCurve {
					return false, nil
				}
			}
		}
	}
	return true, nil
}
