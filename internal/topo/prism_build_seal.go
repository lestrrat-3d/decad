package topo

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const prismSealLimit = 4096

type prismBuildSeal struct {
	payload    *prismPayloadSeal
	walks      *prismWalkSeal
	faces      []prismSealedFace
	components int
	strict     bool
	bounds     measurement.Box
}
type prismSealedFace struct {
	face             *Face
	origin           provenance.Ref
	plane            *surfacegeom.Plane
	cylinder         *surfacegeom.Cylinder
	cap              *CapRecord
	capSource        *CapRecord
	capDenotedSource *big.Rat
	hasAxial         bool
	axial            float64
	reversed         bool
	loops            []prismSealedLoop
}
type prismSealedLoop struct {
	loop  *Loop
	outer bool
	edges []prismSealedEdge
}
type prismSealedEdge struct {
	edge                 *Edge
	forward              bool
	adjacent             [2]*Face
	count                int
	curve                surfacegeom.Curve
	start, end           *Vertex
	startPos, endPos     r3.Vec
	startBound, endBound units.Value
}

// SealPrismBuildFaces records bounded face, cap, surface and adjacency values
// immediately after a native prism build. A failed optional seal leaves no
// witness and does not alter the built body. The caller supplies isolated work.
func (b *Body) SealPrismBuildFaces(faces []*Face, spend func(uint64) error) bool {
	if b == nil || b.prismBuildSeal != nil || spend == nil || len(faces) == 0 || len(faces) > prismSealLimit {
		return false
	}
	count, loops, edges := 0, 0, 0
	for _, f := range faces {
		if spend(1) != nil {
			return false
		}
		count++
		if f == nil || f.Body != b || len(f.Origins) != 1 {
			return false
		}
		switch f.Surface.(type) {
		case surfacegeom.Plane, surfacegeom.Cylinder:
		default:
			return false
		}
		if f.CapRecord != nil && f.CapRecord.ZDenoted != nil && (f.CapRecord.ZDenoted.Num().BitLen() > 4096 || f.CapRecord.ZDenoted.Denom().BitLen() > 4096) {
			return false
		}
		for _, l := range f.Loops {
			if spend(1) != nil {
				return false
			}
			count++
			loops++
			if l == nil || count > prismSealLimit {
				return false
			}
			for _, ce := range l.CoEdges {
				if spend(1) != nil {
					return false
				}
				count++
				edges++
				if ce.Edge == nil || len(ce.Edge.Faces) > 2 || count > prismSealLimit {
					return false
				}
				if ce.Edge.Start == nil || ce.Edge.End == nil {
					return false
				}
				switch ce.Edge.Curve.(type) {
				case surfacegeom.Line3, surfacegeom.Circle3, surfacegeom.Arc3:
				default:
					return false
				}
			}
		}
	}
	// The validation above caps all counts, so this slice initialization charge cannot overflow.
	if spend(1+64*uint64(len(faces))+8*uint64(loops)+64*uint64(edges)) != nil {
		return false
	}
	seal := &prismBuildSeal{faces: make([]prismSealedFace, 0, len(faces)), components: count, bounds: b.Bounds}
	for _, f := range faces {
		// Include the copied surface and cap, plus the rational's actual limbs.
		work := uint64(128)
		if f.CapRecord != nil && f.CapRecord.ZDenoted != nil {
			rat := f.CapRecord.ZDenoted
			work += uint64(len(rat.Num().Bits()) + len(rat.Denom().Bits()))
		}
		if spend(work) != nil {
			return false
		}
		sf := prismSealedFace{face: f, origin: f.Origins[0], hasAxial: f.HasAxialDelta, axial: f.AxialDelta, reversed: f.Reversed, loops: make([]prismSealedLoop, 0, len(f.Loops))}
		switch v := f.Surface.(type) {
		case surfacegeom.Plane:
			sf.plane = &v
		case surfacegeom.Cylinder:
			sf.cylinder = &v
		}
		if f.CapRecord != nil {
			sf.capSource = f.CapRecord
			sf.capDenotedSource = f.CapRecord.ZDenoted
			v := *f.CapRecord
			if v.ZDenoted != nil {
				v.ZDenoted = new(big.Rat).Set(v.ZDenoted)
			}
			sf.cap = &v
		}
		for _, l := range f.Loops {
			if spend(8) != nil {
				return false
			}
			sl := prismSealedLoop{loop: l, outer: l.Outer, edges: make([]prismSealedEdge, 0, len(l.CoEdges))}
			for _, ce := range l.CoEdges {
				if spend(64) != nil {
					return false
				}
				se := prismSealedEdge{edge: ce.Edge, forward: ce.Forward, count: len(ce.Edge.Faces), curve: ce.Edge.Curve,
					start: ce.Edge.Start, end: ce.Edge.End, startPos: ce.Edge.Start.Position, endPos: ce.Edge.End.Position,
					startBound: ce.Edge.Start.Bound, endBound: ce.Edge.End.Bound}
				copy(se.adjacent[:], ce.Edge.Faces)
				sl.edges = append(sl.edges, se)
			}
			sf.loops = append(sf.loops, sl)
		}
		seal.faces = append(seal.faces, sf)
	}
	if spend(1) != nil {
		return false
	}
	seal.strict = true
	b.prismBuildSeal = seal
	return true
}

// HasStrictPrismBuildSeal reports whether both original Build snapshots use
// pointer-bound cap records and bitwise field replay.
func (b *Body) HasStrictPrismBuildSeal() bool {
	return b != nil && b.prismBuildSeal != nil && b.prismBuildSeal.strict && b.prismBuildSeal.payload != nil
}

// ReplayPrismBuiltBounds returns the producer's original box only while the
// current body's box still matches that construction-time snapshot bitwise.
func (b *Body) ReplayPrismBuiltBounds(ctx context.Context,
	budget proof.ReplayKeyBudget) (measurement.Box, bool, error) {
	if budget == nil {
		return measurement.Box{}, false, nil
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return measurement.Box{}, false, err
	}
	if !b.HasStrictPrismBuildSeal() {
		return measurement.Box{}, false, nil
	}
	box := b.prismBuildSeal.bounds
	if !prismSealSameVec(b.Bounds.Min, box.Min) || !prismSealSameVec(b.Bounds.Max, box.Max) ||
		!prismSealSameValue(b.Bounds.Bound, box.Bound) || b.Bounds.Exactness != box.Exactness {
		return measurement.Box{}, false, nil
	}
	return box, true, nil
}

// ReplayPrismBuiltFace checks one face against its immutable Build snapshot.
func (b *Body) ReplayPrismBuiltFace(ctx context.Context, face *Face, budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if b == nil || face == nil || b.prismBuildSeal == nil || !b.prismBuildSeal.strict {
		return false, nil
	}
	seal := b.prismBuildSeal
	if err := budget.ReserveRecords(ctx, uint64(seal.components)); err != nil {
		return false, err
	}
	if err := budget.ReserveBytes(ctx, uint64(seal.components)*128+256); err != nil {
		return false, err
	}
	var match *prismSealedFace
	for i := range seal.faces {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if seal.faces[i].face == face {
			if match != nil {
				return false, nil
			}
			match = &seal.faces[i]
		}
	}
	if match == nil {
		return false, nil
	}
	// Cover the fixed face, surface, frame and cap field comparisons.
	if err := budget.SpendWork(ctx, 128); err != nil {
		return false, err
	}
	if face.Body != b || len(face.Origins) != 1 || face.Origins[0] != match.origin ||
		face.HasAxialDelta != match.hasAxial || !prismSealSameFloat(face.AxialDelta, match.axial) ||
		face.Reversed != match.reversed || len(face.Loops) != len(match.loops) {
		return false, nil
	}
	if match.plane != nil {
		v, ok := face.Surface.(surfacegeom.Plane)
		if !ok || !prismSealSameFrame(v.Frame, match.plane.Frame) {
			return false, nil
		}
	} else {
		v, ok := face.Surface.(surfacegeom.Cylinder)
		if !ok || match.cylinder == nil || !prismSealSameVec(v.Origin, match.cylinder.Origin) ||
			!prismSealSameVec(v.Axis, match.cylinder.Axis) || !prismSealSameValue(v.Radius, match.cylinder.Radius) {
			return false, nil
		}
	}
	if face.CapRecord != match.capSource || (face.CapRecord == nil) != (match.cap == nil) {
		return false, nil
	}
	if match.cap != nil {
		a, c := face.CapRecord, match.cap
		if !prismSealSameFrame(a.Frame, c.Frame) || !prismSealSameTransform(a.Xform, c.Xform) ||
			!prismSealSameFloat(a.Z, c.Z) || a.ZDenoted != match.capDenotedSource ||
			(a.ZDenoted == nil) != (c.ZDenoted == nil) {
			return false, nil
		}
		if a.ZDenoted != nil {
			cmp, err := proof.ReplayRatCompare(ctx, a.ZDenoted, c.ZDenoted, budget)
			if err != nil || cmp != 0 {
				return false, err
			}
		}
	}
	for i, sl := range match.loops {
		if err := budget.SpendWork(ctx, 4); err != nil {
			return false, err
		}
		l := face.Loops[i]
		if l != sl.loop || l == nil || l.Outer != sl.outer || len(l.CoEdges) != len(sl.edges) {
			return false, nil
		}
		for j, se := range sl.edges {
			// Cover endpoints, bounds, orientation and the largest curve comparison.
			if err := budget.SpendWork(ctx, 64); err != nil {
				return false, err
			}
			ce := l.CoEdges[j]
			if ce.Edge != se.edge || ce.Edge == nil || ce.Forward != se.forward || len(ce.Edge.Faces) != se.count {
				return false, nil
			}
			if ce.Edge.Start != se.start || ce.Edge.End != se.end || ce.Edge.Start == nil || ce.Edge.End == nil ||
				!prismSealSameVec(ce.Edge.Start.Position, se.startPos) ||
				!prismSealSameVec(ce.Edge.End.Position, se.endPos) ||
				!prismSealSameValue(ce.Edge.Start.Bound, se.startBound) ||
				!prismSealSameValue(ce.Edge.End.Bound, se.endBound) {
				return false, nil
			}
			switch want := se.curve.(type) {
			case surfacegeom.Line3:
				_, ok := ce.Edge.Curve.(surfacegeom.Line3)
				if !ok {
					return false, nil
				}
			case surfacegeom.Circle3:
				got, ok := ce.Edge.Curve.(surfacegeom.Circle3)
				if !ok || !prismSealSameCurve(got.Center, got.Axis, got.Radius,
					want.Center, want.Axis, want.Radius) {
					return false, nil
				}
			case surfacegeom.Arc3:
				got, ok := ce.Edge.Curve.(surfacegeom.Arc3)
				if !ok || !prismSealSameCurve(got.Center, got.Axis, got.Radius,
					want.Center, want.Axis, want.Radius) {
					return false, nil
				}
			default:
				return false, nil
			}

			for k := range se.count {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				if ce.Edge.Faces[k] != se.adjacent[k] {
					return false, nil
				}
			}
		}
	}
	return true, nil
}

type prismPayloadSeal struct {
	frame                                  r3.Frame
	xform                                  r3.Transform
	z0, z1, z0Delta, z1Delta, sectionDelta float64
	z0Denoted, z1Denoted                   prismplacement.LevelDenotationSnapshot
	profiles                               []momentinput.Profile
}

// SealPrismBuildPayload binds the supported analytic section records and
// placement to the face snapshot. Unsupported records leave no payload seal.
func (b *Body) SealPrismBuildPayload(frame r3.Frame, xform r3.Transform, z0, z1, z0Delta, z1Delta, sectionDelta float64,
	z0Denoted, z1Denoted prismplacement.LevelDenotation,
	profiles []momentinput.Profile, spend func(uint64) error) bool {
	if b == nil || b.prismBuildSeal == nil || b.prismBuildSeal.payload != nil || spend == nil || len(profiles) == 0 || z0Delta != 0 || z1Delta != 0 {
		return false
	}
	z0Bytes, ok := prismplacement.LevelDenotationSnapshotCost(z0Denoted)
	if !ok {
		return false
	}
	z1Bytes, ok := prismplacement.LevelDenotationSnapshotCost(z1Denoted)
	if !ok || spend(128+z0Bytes+z1Bytes) != nil {
		return false
	}
	count := 0
	for _, p := range profiles {
		if spend(1) != nil {
			return false
		}
		count++
		if count > prismSealLimit {
			return false
		}
		for li := -1; li < len(p.Holes); li++ {
			if spend(1) != nil {
				return false
			}
			count++
			if count > prismSealLimit {
				return false
			}
			loop := p.Outer
			if li >= 0 {
				loop = p.Holes[li]
			}
			for _, seg := range loop.Segments {
				if spend(1) != nil {
					return false
				}
				count++
				if count > prismSealLimit {
					return false
				}
				switch seg.(type) {
				case sectionrecord.LineSeg, sectionrecord.CircleSeg, sectionrecord.ArcSeg:
				default:
					return false
				}
			}
		}
	}
	copyLoop := func(in sectionrecord.LoopRecord) (sectionrecord.LoopRecord, bool) {
		if spend(uint64(len(in.Segments))) != nil {
			return sectionrecord.LoopRecord{}, false
		}
		return sectionrecord.LoopRecord{Segments: append([]sectionrecord.CurveSegment(nil), in.Segments...)}, true
	}
	if spend(uint64(len(profiles))) != nil {
		return false
	}
	sealed := &prismPayloadSeal{frame: frame, xform: xform, z0: z0, z1: z1,
		z0Delta: z0Delta, z1Delta: z1Delta, sectionDelta: sectionDelta,
		z0Denoted: prismplacement.SnapshotLevelDenotation(z0Denoted),
		z1Denoted: prismplacement.SnapshotLevelDenotation(z1Denoted),
		profiles:  make([]momentinput.Profile, len(profiles))}
	for i, p := range profiles {
		if spend(1) != nil {
			return false
		}
		outer, ok := copyLoop(p.Outer)
		if !ok {
			return false
		}
		sealed.profiles[i].Outer = outer
		if spend(uint64(len(p.Holes))) != nil {
			return false
		}
		sealed.profiles[i].Holes = make([]sectionrecord.LoopRecord, len(p.Holes))
		for j, h := range p.Holes {
			if spend(1) != nil {
				return false
			}
			loop, ok := copyLoop(h)
			if !ok {
				return false
			}
			sealed.profiles[i].Holes[j] = loop
		}
	}
	b.prismBuildSeal.payload = sealed
	return true
}

// ReplayPrismBuildPayload verifies a current supported payload against its
// original Build snapshot, with all segment comparisons charged to budget.
func (b *Body) ReplayPrismBuildPayload(ctx context.Context, frame r3.Frame, xform r3.Transform, z0, z1, z0Delta, z1Delta, sectionDelta float64,
	z0Denoted, z1Denoted prismplacement.LevelDenotation,
	profiles []momentinput.Profile, budget proof.ReplayKeyBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if b == nil || b.prismBuildSeal == nil || !b.prismBuildSeal.strict || b.prismBuildSeal.payload == nil {
		return false, nil
	}
	s := b.prismBuildSeal.payload
	if !prismSealSameFrame(frame, s.frame) || !prismSealSameTransform(xform, s.xform) ||
		!prismSealSameFloat(z0, s.z0) || !prismSealSameFloat(z1, s.z1) ||
		!prismSealSameFloat(z0Delta, s.z0Delta) || !prismSealSameFloat(z1Delta, s.z1Delta) ||
		!prismSealSameFloat(sectionDelta, s.sectionDelta) || len(profiles) != len(s.profiles) {
		return false, nil
	}
	match, err := s.z0Denoted.ReplayMatches(ctx, z0Denoted, budget)
	if err != nil || !match {
		return false, err
	}
	match, err = s.z1Denoted.ReplayMatches(ctx, z1Denoted, budget)
	if err != nil || !match {
		return false, err
	}
	checkLoop := func(a, c sectionrecord.LoopRecord) (bool, error) {
		if len(a.Segments) != len(c.Segments) {
			return false, nil
		}
		for i, segment := range a.Segments {
			if err := budget.SpendWork(ctx, 64); err != nil {
				return false, err
			}
			if !sectionrecord.IdenticalAnalyticSegment(segment, c.Segments[i]) {
				return false, nil
			}
		}
		return true, nil
	}
	for i, p := range profiles {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		want := s.profiles[i]
		if len(p.Holes) != len(want.Holes) {
			return false, nil
		}
		ok, err := checkLoop(p.Outer, want.Outer)
		if err != nil || !ok {
			return false, err
		}
		for j, h := range p.Holes {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			ok, err = checkLoop(h, want.Holes[j])
			if err != nil || !ok {
				return false, err
			}
		}
	}
	return true, nil
}

func prismSealSameFloat(a, b float64) bool { return math.Float64bits(a) == math.Float64bits(b) }

func prismSealSameVec(a, b r3.Vec) bool {
	return prismSealSameFloat(a.X, b.X) && prismSealSameFloat(a.Y, b.Y) && prismSealSameFloat(a.Z, b.Z)
}

func prismSealSameFrame(a, b r3.Frame) bool {
	return prismSealSameVec(a.Origin(), b.Origin()) &&
		prismSealSameVec(a.U(), b.U()) && prismSealSameVec(a.V(), b.V())
}

func prismSealSameTransform(a, b r3.Transform) bool {
	x, y := a.Basis(), b.Basis()
	return prismSealSameVec(x.EX, y.EX) && prismSealSameVec(x.EY, y.EY) &&
		prismSealSameVec(x.EZ, y.EZ) && prismSealSameVec(a.Translation(), b.Translation())
}

func prismSealSameValue(a, b units.Value) bool {
	return a.Unit() == b.Unit() && prismSealSameFloat(a.Mag(), b.Mag())
}

func prismSealSameCurve(ac, aa r3.Vec, ar units.Value, bc, ba r3.Vec, br units.Value) bool {
	return prismSealSameVec(ac, bc) && prismSealSameVec(aa, ba) && prismSealSameValue(ar, br)
}
