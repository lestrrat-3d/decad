package rimextension

import (
	"context"
	"math"
	"math/big"
	"strconv"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// The prism is extruded toward negative world Y. Only its rightmost side
// may enter the window. Both native endpoint radii bound the closing chord.
func transversePrism(ctx context.Context, a Admission, w SourceWindow, m *proofbound.ReplayBudget) (*big.Rat, bool, error) {
	if a.Cap.Body != a.Side.Body || a.Cap.Translation != (r3.Vec{}) || a.Side.Translation != (r3.Vec{}) {
		return nil, false, nil
	}
	p, ok, err := windowPrism(ctx, a.Side, m)
	if !ok || err != nil {
		return nil, false, err
	}
	z0, z1 := p.DenotedZ0(), p.DenotedZ1()
	if z0 == nil || z1 == nil {
		return nil, false, nil
	}
	if windowRole(a.Side, provenance.RoleCapEnd) {
		z0, z1 = z1, z0
	} else if !windowRole(a.Side, provenance.RoleCapStart) {
		return nil, false, nil
	}
	x := scalar(a.Section.Stop.Y)
	y := scalar(a.Section.Stop.Z)
	lo, hi := scalar(w.XMin), scalar(w.XMax)
	o := lift(p.Frame.Origin())
	trueY := minus(o[1], z0)
	if wabs(minus(trueY, y)).Cmp(divide(scalar(a.Setback), scalar(1000))) >= 0 || minus(o[1], z1).Cmp(plus(y, scalar(w.OffsetMin))) >= 0 {
		return nil, false, nil
	}
	found := false
	for i, seg := range p.Profile.Outer.Segments {
		if err := m.ReserveBundle(ctx, 96, 1, 32768); err != nil {
			return nil, false, err
		}
		switch s := seg.(type) {
		case momentinput.LineSeg:
			if !wholeShoulderWalk(s.TStart, s.TEnd) {
				return nil, false, nil
			}
			ax, bx := plus(o[0], scalar(s.Start.U)), plus(o[0], scalar(s.End.U))
			az, bz := plus(o[2], scalar(s.Start.V)), plus(o[2], scalar(s.End.V))
			if ax.Cmp(x) > 0 || bx.Cmp(x) > 0 {
				return nil, false, nil
			}
			if windowRole(a.Cap, "side(0,"+strconv.Itoa(i)+")") {
				if found || ax.Cmp(x) != 0 || bx.Cmp(x) != 0 || wmin(az, bz).Cmp(lo) >= 0 || wmax(az, bz).Cmp(hi) <= 0 {
					return nil, false, nil
				}
				found = true
			} else if wmin(az, bz).Cmp(hi) <= 0 && wmax(az, bz).Cmp(lo) >= 0 && wmax(ax, bx).Cmp(scalar(w.YMin)) >= 0 {
				return nil, false, nil
			}
		case momentinput.ArcSeg:
			if !wholeShoulderWalk(s.TStart, s.TEnd) {
				return nil, false, nil
			}
			cx, cz := plus(o[0], scalar(s.Center.U)), plus(o[2], scalar(s.Center.V))
			dx := minus(scalar(w.YMin), cx)
			if dx.Sign() <= 0 {
				return nil, false, nil
			}
			dz := wmax(scalar(0), wmax(minus(lo, cz), minus(cz, hi)))
			for _, pt := range []momentinput.Point2{s.Start, s.End} {
				u, v := minus(scalar(pt.U), scalar(s.Center.U)), minus(scalar(pt.V), scalar(s.Center.V))
				if plus(times(dx, dx), times(dz, dz)).Cmp(plus(times(u, u), times(v, v))) <= 0 {
					return nil, false, nil
				}
			}
		default:
			return nil, false, nil
		}
	}
	return trueY, found, nil
}

// The retained swept wall must cover the complete edit and witness box in
// its axial direction. Its oriented quadrant defines a single-valued graph.
func transverseCylinder(a Admission, wing int, w SourceWindow) (windowNative, bool) {
	var n windowNative
	f := a.Boss
	p, ok := f.Body.Payload.(breprecord.Payload)
	if !ok || p.Xform != r3.Identity() || f.Translation != (r3.Vec{}) || p.CarrierCharge != 0 || wing != 1 && wing != -1 {
		return n, false
	}
	c, ok := f.Native.Surface.(surfacegeom.Cylinder)
	if !ok || c.Axis != (r3.Vec{X: 1}) || f.Final.Reversed || f.Native.Reversed {
		return n, false
	}
	for i, fr := range p.Faces {
		if len(f.Native.Origins) != 1 || !breprecord.MatchRecordFace(f.Body, f.Native, nil, i) {
			continue
		}
		s, ok := fr.Wall.(momentinput.ArcSeg)
		if !ok || fr.Region != nil || fr.Delta != 0 || fr.Z0Delta != 0 || fr.Z1Delta != 0 || fr.Frame.U() != (r3.Vec{Y: 1}) || fr.Frame.V() != (r3.Vec{Z: 1}) || s.TStart != 0 || s.TEnd != 1 {
			return n, false
		}
		r := scalar(c.Radius.Base())
		if r == nil || r.Sign() <= 0 {
			return n, false
		}
		u, v := scalar(s.Center.U), scalar(s.Center.V)
		ax, az, bx, bz := minus(scalar(s.Start.U), u), signed(minus(scalar(s.Start.V), v), wing), minus(scalar(s.End.U), u), signed(minus(scalar(s.End.V), v), wing)
		if wing < 0 {
			ax, az, bx, bz = bx, bz, ax, az
		}
		if ax.Cmp(r) != 0 || az.Sign() != 0 || bx.Sign() != 0 || bz.Cmp(r) != 0 {
			return n, false
		}
		o := lift(fr.Frame.Origin())
		x0, x1 := plus(o[0], scalar(fr.Z0)), plus(o[0], scalar(fr.Z1))
		if x0.Cmp(scalar(w.YMin)) >= 0 || x1.Cmp(scalar(w.YMax)) <= 0 {
			return n, false
		}
		n.cx, n.cz, n.r, n.out = plus(o[2], v), plus(o[1], u), r, 1
		if n.cx.Cmp(scalar(c.Origin.Z)) != 0 || n.cz.Cmp(scalar(c.Origin.Y)) != 0 {
			return n, false
		}
		return n, true
	}
	return n, false
}

// transverseWitness lifts an interior point of a held triangle. Its complete
// displacement ball must remain in the desired connected source cell.
func transverseWitness(ctx context.Context, mesh Mesh, beta []float64, id int, planar bool, w SourceWindow, n windowNative, stop point, wing int, setback *big.Rat, m *proofbound.ReplayBudget) (bool, error) {
	var pt point
	var i, j, k int
	if planar {
		i, j, k = 0, 1, 2
		pt[i] = minus(stop[0], signed(times(setback, scalar(.6)), wing))
		pt[j] = minus(stop[1], times(setback, scalar(.5)))
	} else {
		i, j, k = 1, 2, 0
		pt[i] = plus(stop[1], times(setback, scalar(1.0/3)))
		pt[j] = minus(stop[2], times(setback, scalar(.5)))
	}
	det := func(a, b, c point) *big.Rat {
		return minus(times(minus(b[i], a[i]), minus(c[j], a[j])), times(minus(b[j], a[j]), minus(c[i], a[i])))
	}
	low := [3]*big.Rat{scalar(w.XMin), scalar(w.YMin), plus(stop[2], scalar(w.OffsetMin))}
	high := [3]*big.Rat{scalar(w.XMax), scalar(w.YMax), plus(stop[2], scalar(w.OffsetMax))}
	for t, ids := range mesh.Triangles {
		if err := m.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		if mesh.Sources[t] != id {
			continue
		}
		if err := m.ReserveBundle(ctx, 128, 1, 16384); err != nil {
			return false, err
		}
		a, b, c := lift(mesh.Vertices[ids[0]]), lift(mesh.Vertices[ids[1]]), lift(mesh.Vertices[ids[2]])
		d := det(a, b, c)
		if d.Sign() == 0 {
			continue
		}
		p, q, r := det(b, c, pt), det(c, a, pt), det(a, b, pt)
		if p.Sign()*d.Sign() < 0 || q.Sign()*d.Sign() < 0 || r.Sign()*d.Sign() < 0 {
			continue
		}
		pt[k] = divide(plus(times(p, a[k]), plus(times(q, b[k]), times(r, c[k]))), d)
		errBound := scalar(max(beta[ids[0]], beta[ids[1]], beta[ids[2]]))
		inside := true
		for j := range 3 {
			if minus(pt[j], errBound).Cmp(low[j]) <= 0 || plus(pt[j], errBound).Cmp(high[j]) >= 0 {
				inside = false
			}
		}
		u, v := signed(minus(pt[0], n.cx), wing), minus(pt[2], n.cz)
		if !inside || signed(minus(stop[0], pt[0]), wing).Cmp(errBound) <= 0 || u.Cmp(errBound) <= 0 || v.Cmp(errBound) <= 0 {
			continue
		}
		if planar {
			u, v = minus(u, errBound), minus(v, errBound)
			if minus(stop[1], pt[1]).Cmp(errBound) <= 0 || plus(times(u, u), times(v, v)).Cmp(times(n.r, n.r)) <= 0 {
				continue
			}
		} else if minus(pt[1], stop[1]).Cmp(errBound) <= 0 {
			continue
		}
		return true, nil
	}
	return false, nil
}

// PrepareTransverse binds the finite old bevel to the two native stock
// planes and the finite outward cylindrical quadrant before changing the mesh.
func PrepareTransverse(ctx context.Context, env topo.Env, a Admission, c Candidate, m *proofbound.ReplayBudget) (Prepared, bool, error) {
	var none Prepared
	h, ok := newChart(a.Section)
	if !ok || !c.Groups.OpenCylinderEnd || len(a.Stop) != 2 || a.Stop[0].Body != a.Stop[1].Body || a.Stop[0].Translation != (r3.Vec{}) || a.Stop[1].Translation != (r3.Vec{}) || a.Stop[0].Native == a.Stop[1].Native {
		return none, false, nil
	}
	d := scalar(a.Setback)
	if d == nil || d.Sign() <= 0 {
		return none, false, nil
	}
	start, end := h.start, h.end
	wing := start[0].Cmp(end[0])
	if wing == 0 {
		return none, false, nil
	}
	in := minus(end[0], signed(d, wing))
	outside := plus(start[0], signed(divide(d, scalar(32)), wing))
	w := SourceWindow{Side0: a.Section.Stop, Side1: a.Section.End, XMin: proofbound.RatFloatDown(wmin(in, outside)), XMax: proofbound.RatFloatUp(wmax(in, outside)), YMin: proofbound.RatFloatDown(minus(start[1], times(d, scalar(1.16)))), YMax: proofbound.RatFloatUp(plus(start[1], divide(d, scalar(2)))), OffsetMin: proofbound.RatFloatDown(times(d, scalar(-1.07))), OffsetMax: proofbound.RatFloatUp(times(d, scalar(.1)))}
	trueY, ok, err := transversePrism(ctx, a, w, m)
	if !ok || err != nil {
		return none, false, err
	}
	bp, ok := a.Boss.Body.Payload.(breprecord.Payload)
	if !ok || len(a.Boss.Native.Origins) != 1 {
		return none, false, nil
	}
	if err := m.SpendWork(ctx, uint64(4*len(bp.Faces)+1)); err != nil {
		return none, false, err
	}
	n, ok := transverseCylinder(a, wing, w)
	if !ok {
		return none, false, nil
	}
	p, ok := a.Stop[0].Body.Payload.(loftrecord.Payload)
	if !ok || !p.MatchesDirectSections(a.Stop[0].Body) || p.JoinDelta != 0 || p.JoinPlaneGap != 0 {
		return none, false, nil
	}
	stop, toe, heel, ok, err := TransverseLoftStop(ctx, &p, wing, trueY, start[1], scalar(w.YMin), scalar(w.YMax), m)
	if !ok || err != nil {
		return none, false, err
	}
	ok, err = transverseStopRoles(ctx, a, c, p, wing, stop, toe, heel, trueY, start[1], scalar(w.YMax), m)
	if !ok || err != nil {
		return none, false, err
	}
	delta := plus(wabs(minus(trueY, start[2])), wabs(minus(start[0], stop)))
	for _, q := range []*big.Rat{minus(toe, scalar(a.Section.Heel.Y)), minus(heel, scalar(a.Section.Toe.Z)), minus(toe, minus(start[1], d)), minus(heel, minus(trueY, d))} {
		delta = plus(delta, wabs(q))
	}
	if delta.Cmp(divide(d, scalar(1000))) >= 0 {
		return none, false, nil
	}
	loY, hiY := minus(plus(start[2], scalar(w.OffsetMin)), n.cz), minus(plus(start[2], scalar(w.OffsetMax)), n.cz)
	innerZ, outerZ := signed(minus(in, n.cx), wing), signed(minus(stop, n.cx), wing)
	sq := func(a *big.Rat) *big.Rat { return times(a, a) }
	if loY.Sign() <= 0 || hiY.Cmp(n.r) >= 0 || innerZ.Sign() <= 0 || plus(sq(innerZ), sq(hiY)).Cmp(sq(n.r)) >= 0 || outerZ.Sign() <= 0 || plus(sq(outerZ), sq(loY)).Cmp(sq(n.r)) <= 0 {
		return none, false, nil
	}
	s, ok, err := loadStock(ctx, a.Stock, m)
	if !ok || err != nil {
		return none, false, err
	}
	ids := map[int]bool{}
	capID, bossID := -1, -1
	for i, f := range s.fs {
		switch f {
		case c.Cap, c.Side, c.Boss, c.Stops[0], c.Stops[1]:
			ids[i] = true
		}
		if f == c.Side {
			capID = i
		}
		if f == c.Boss {
			bossID = i
		}
	}
	if len(ids) != 5 {
		return none, false, nil
	}
	if err := m.ReserveBundle(ctx, uint64(len(s.p.Verts))*4, uint64(len(s.p.Verts)), uint64(len(s.p.Verts))*24); err != nil {
		return none, false, err
	}
	v := make([]r3.Vec, len(s.p.Verts))
	for i, p := range s.p.Verts {
		v[i] = TransversePoint(p)
	}
	ok, err = WindowSourcesCovered(ctx, v, s.p.Tris, s.p.FaceOf, s.p.VertexBound, ids, w, m)
	if !ok || err != nil {
		return none, false, err
	}
	mesh := Mesh{v, s.p.Tris, s.p.FaceOf}
	for _, id := range []int{capID, bossID} {
		ok, err = transverseWitness(ctx, mesh, s.p.VertexBound, id, id == capID, w, n, start, wing, d, m)
		if !ok || err != nil {
			return none, false, err
		}
	}
	mesh.Sources = s.p.Src
	out, ok, err := Apply(ctx, mesh, c.Section, c.Groups, c.StopFacets, w, m)
	if !ok || err != nil {
		return none, false, err
	}
	radial, ok, err := cylinderExtensionBound(ctx, c.Section, n, out, c.Groups.Extension, m)
	if !ok || err != nil {
		return none, false, err
	}
	// Bound the endpoint mismatch at fixed Y. Both longitudinal ordinates
	// have the same sign and the true ordinate exceeds innerZ throughout.
	dx, dy := minus(end[0], n.cx), minus(trueY, n.cz)
	term := divide(wabs(minus(plus(sq(dx), sq(dy)), sq(n.r))), plus(wabs(dx), innerZ))
	// Over the rectangle the cylinder graph has slope |y-yc|/|z-zc|.
	// Its exposed area is at most (1+slope) times its projected area.
	slope := divide(hiY, innerZ)
	span := wabs(minus(outside, in))
	wide := minus(scalar(w.YMax), scalar(w.YMin))
	deep := minus(scalar(w.OffsetMax), scalar(w.OffsetMin))
	area := plus(times(span, plus(wide, deep)), times(plus(scalar(1), slope), times(wide, deep)))
	bounds := WindowBounds{Deviation: proofbound.RatFloatUp(plus(times(plus(scalar(4), slope), delta), term)), Area: proofbound.RatFloatUp(times(scalar(4), area)), Volume: proofbound.RatFloatUp(times(span, times(wide, deep))), support: w, coveredFaces: ids}
	if math.IsInf(bounds.Deviation, 0) || math.IsInf(bounds.Area, 0) || math.IsInf(bounds.Volume, 0) {
		return none, false, nil
	}
	for i, p := range out.Vertices {
		out.Vertices[i] = r3.NewVec(p.Y, p.Z, p.X)
	}
	return prepareOutput(ctx, env, a, s.p, out, radial, bounds, 2, 0, m)
}

// Each selected Loft role names one held triangle. Resolve it to its own
// paired section segment before checking the stop and sloping wall join.
func transverseStopRoles(ctx context.Context, a Admission, c Candidate, p loftrecord.Payload, wing int,
	station, toe, heel, capY, sideX, highX *big.Rat, m *proofbound.ReplayBudget) (bool, error) {
	if len(a.Stop) != 2 || len(p.Segment) != 1 || len(p.Segment[0]) == 0 || len(p.Segment[0]) > 512 || len(p.Cell) != p.Walls || len(p.Side) != p.Walls || p.Walls != 2*len(p.Segment[0]) {
		return false, nil
	}
	if err := m.ReserveBundle(ctx, uint64(16*p.Walls), uint64(p.Walls), uint64(256*p.Walls)); err != nil {
		return false, err
	}
	segments := [2]int{-1, -1}
	for _, f := range a.Stop {
		idx := 1
		if f.Final == c.StopWall {
			idx = 0
		}
		if segments[idx] >= 0 || len(f.Native.Origins) != 1 {
			return false, nil
		}
		for k, cell := range p.Cell {
			if cell[0] != 0 || cell[1] < 0 || cell[1] >= len(p.Segment[0]) || p.Side[k] > 1 {
				return false, nil
			}
			role := "side(0," + strconv.Itoa(cell[1]) + "," + strconv.Itoa(int(p.Side[k])) + ")"
			if !windowRole(f, role) {
				continue
			}
			if segments[idx] >= 0 {
				return false, nil
			}
			segments[idx] = p.Segment[0][cell[1]]
		}
	}
	if segments[0] == segments[1] {
		return false, nil
	}
	var x, y [2]*big.Rat
	for k, profile := range [2]momentinput.Profile{p.Profile0, p.Profile1} {
		frame := [2]r3.Frame{p.Frame0, p.Frame1}[k]
		y[k] = scalar(frame.Origin().Y)
		var ends [2][2]point
		for i, seg := range segments {
			if seg < 0 || seg >= len(profile.Outer.Segments) {
				return false, nil
			}
			line, ok := profile.Outer.Segments[seg].(momentinput.LineSeg)
			if !ok || !wholeShoulderWalk(line.TStart, line.TEnd) {
				return false, nil
			}
			for j, v := range []momentinput.Point2{line.Start, line.End} {
				ends[i][j] = point{plus(scalar(frame.Origin().X), scalar(v.U)), y[k], plus(scalar(frame.Origin().Z), signed(scalar(v.V), wing))}
			}
		}
		sa, sb := ends[0][0], ends[0][1]
		ba, bb := ends[1][0], ends[1][1]
		if sa[0].Cmp(sb[0]) > 0 {
			sa, sb = sb, sa
		}
		if ba[2].Cmp(station) != 0 {
			ba, bb = bb, ba
		}
		if sa[2].Cmp(station) != 0 || sb[2].Cmp(station) != 0 || sb[0].Cmp(highX) <= 0 || sa[0].Cmp(ba[0]) != 0 || ba[0].Cmp(bb[0]) != 0 || ba[2].Cmp(station) != 0 || signed(minus(bb[2], station), wing).Sign() <= 0 {
			return false, nil
		}
		x[k] = sa[0]
	}
	if x[1].Cmp(x[0]) == 0 || y[0].Cmp(y[1]) == 0 {
		return false, nil
	}
	at := func(h, a, b, c, d *big.Rat) *big.Rat {
		return plus(c, times(divide(minus(h, a), minus(b, a)), minus(d, c)))
	}
	return at(capY, y[0], y[1], x[0], x[1]).Cmp(toe) == 0 && at(sideX, x[0], x[1], y[0], y[1]).Cmp(heel) == 0, nil
}
