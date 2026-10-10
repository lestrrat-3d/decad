package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file publishes docs/helix-design.md Table CB's topology over the held
// shell internal/coilshell builds, and Table CM's four readings from the record's
// closed forms. Interior stations are mesh vertices, never topology: a wall
// is ONE Faceted face per profile segment spanning every turn, bounded by two
// rim edges and two helix edges.

// coilRecordOfPayload supplies the resolved axis and recorded sweep to the shell builder.
func coilRecordOfPayload(cp coilPayload) (coilshell.Record, error) {
	return coilshell.Read(coilshell.Input{
		Profile: cp.profile, Frame: cp.frame, Transform: cp.xform,
		Axis: coilshell.AxisInput{
			AU: cp.line.AU, AV: cp.line.AV, AUBound: cp.line.AUBound, AVBound: cp.line.AVBound,
			DU: cp.line.DU, DV: cp.line.DV, DUBound: cp.line.DUBound, DVBound: cp.line.DVBound,
			Side: cp.side,
		},
		Pitch: cp.pitch, Turns: cp.turns, LeftHand: cp.leftHand,
		StationsPerTurn: coilStationsPerTurn, MaxStations: maxCoilStations,
		ArcChordsPerTurn: coilArcChordsPerTurn,
	})
}

// evalCoil builds the body for one payload record. The first build and
// every placement run it.
func evalCoil(ctx context.Context, d *Document, ref producerID, cp coilPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	rec, err := coilRecordOfPayload(cp)
	if err != nil {
		return nil, err
	}
	sh, err := coilshell.Build(ctx, rec)
	if err != nil {
		return nil, err
	}
	m, err := coilMoments(ctx, cp, rec)
	if err != nil {
		return nil, err
	}
	walls, err := coilWallAreas(ctx, rec)
	if err != nil {
		return nil, err
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	faces, err := buildCoilTopology(ctx, body, ref, rec, sh, m.Area, walls)
	if err != nil {
		return nil, err
	}
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	if err := publishCoilReadings(body, rec, sh, m, walls); err != nil {
		return nil, err
	}

	cp.verts, cp.vertexBound, cp.tris, cp.delta, cp.maxRound = sh.Verts, sh.VertexBound, sh.Tris, sh.Delta, sh.MaxRound
	body.payload = cp
	return body, nil
}

// coilMoments is Table CM's region integrals A_Ω, Q, I and M about the
// axis. A polygon reads its exact shoelace sums over the recorded vertices
// (coil.RegionMoments). A profile with an arc reads the recorded region's
// plane-origin integrals, the ones Revolve reads, each enclosed by its own
// proven bound, re-referenced into the axis frame over intervals.
func coilMoments(ctx context.Context, cp coilPayload, rec coilshell.Record) (coil.Moments, error) {
	if !rec.Profile.HasArcs() {
		area := coil.PolygonArea(rec.U, rec.V, rec.LoopIdx)
		return coil.RegionMoments(rec.Rho, rec.Zeta, rec.LoopIdx, area, rec.Axis.Side), nil
	}
	ig, err := cp.profile.EvaluatorIntegralsContext(ctx, freeform.MomentSecondOrder, freeform.NewFreeformWork())
	if err != nil {
		return coil.Moments{}, err
	}
	var pm coil.PlaneMoments
	for _, f := range []struct {
		dst          *coil.Iv
		value, bound float64
	}{
		{&pm.Area, ig.Area, ig.AreaBound},
		{&pm.Mu, ig.Mu, ig.MuBound},
		{&pm.Mv, ig.Mv, ig.MvBound},
		{&pm.Muu, ig.Muu, ig.MuuBound},
		{&pm.Muv, ig.Muv, ig.MuvBound},
		{&pm.Mvv, ig.Mvv, ig.MvvBound},
	} {
		iv, ok := coil.Measured(f.value, f.bound)
		if !ok {
			return coil.Moments{}, fmt.Errorf(`%w: the coil profile's region integrals are not finite`, ErrUnsupported)
		}
		*f.dst = iv
	}
	return rec.Axis.Moments(pm), nil
}

// coilWallAreas encloses every recorded segment's wall area (Table CM's
// Area row), in rec.Profile.Segments order, each widened by the denoted
// map's defect: a line's closed form over its two ends, an arc's or a
// circle's §11.1 bracket.
func coilWallAreas(ctx context.Context, rec coilshell.Record) ([]coil.Iv, error) {
	segs := rec.Profile.Segments
	walls := make([]coil.Iv, len(segs))
	for i, seg := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if seg.IsArc() {
			a, ok, err := coil.ArcArea(ctx, seg, rec.Axis, rec.Pitch, rec.Turns)
			if err != nil {
				return nil, err
			}
			if !ok {
				return nil, fmt.Errorf(`%w: the coil wall of profile loop %d segment %d has no area enclosure`, ErrUnsupported, seg.Loop, seg.Index)
			}
			walls[i] = rec.Stretched(a)
			continue
		}
		v, w := seg.First, rec.Profile.Segments[rec.Profile.Next(i)].First
		a, ok := coil.SegmentArea(rec.Rho[v], rec.Zeta[v], rec.Rho[w], rec.Zeta[w], rec.Pitch, rec.Turns)
		if !ok {
			return nil, fmt.Errorf(`%w: the coil wall of profile loop %d segment %d has no area enclosure`, ErrUnsupported, seg.Loop, seg.Index)
		}
		walls[i] = rec.Stretched(a)
	}
	return walls, nil
}

// publishCoilReadings is Table CM: Volume = |det L|·Θ·Q; the centroid's
// closed form; Area = 2·A_Ω plus every wall's enclosure, widened by L's
// defect; Bounds over the held table widened by δ. Each closed form is an
// exact enclosure rounded once.
func publishCoilReadings(body *Body, rec coilshell.Record, sh coilshell.Shell, m coil.Moments, walls []coil.Iv) error {
	vol, volBound, err := coilshell.Held(coilshell.Volume(rec, m), "volume")
	if err != nil {
		return err
	}
	body.volume = Measurement{
		Value:     units.CubicMillimeters(vol),
		Exactness: exactnessOf(volBound),
		Bound:     units.CubicMillimeters(volBound),
	}

	cr, ct, cn, ok := coil.CentroidCoefficients(m, rec.Pitch, rec.Turns, rec.Sigma)
	if !ok {
		return fmt.Errorf(`%w: the coil's first moment is not proven positive`, ErrUnsupported)
	}
	erU, erV := rec.Axis.Radial()
	x := proofbound.IntervalAdd(rec.Axis.AU, proofbound.IntervalAdd(proofbound.IntervalMul(cr, erU), proofbound.IntervalMul(cn, rec.Axis.DU)))
	y := proofbound.IntervalAdd(rec.Axis.AV, proofbound.IntervalAdd(proofbound.IntervalMul(cr, erV), proofbound.IntervalMul(cn, rec.Axis.DV)))
	z := proofbound.IntervalScale(ct, big.NewRat(int64(rec.Axis.Side), 1))
	c := rec.World.Point(x, y, z)
	var cv [3]float64
	worst := 0.0
	for axis := range 3 {
		held, bound, err := coilshell.Held(c[axis], "centroid")
		if err != nil {
			return err
		}
		cv[axis] = held
		worst = math.Max(worst, bound)
	}
	centroidBound := proofbound.Radius3D(worst)
	body.centroid = VecMeasurement{
		Value:     r3.NewVec(cv[0], cv[1], cv[2]),
		Exactness: exactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	total := rec.Stretched(proofbound.IntervalScale(m.Area, big.NewRat(2, 1)))
	for _, w := range walls {
		total = proofbound.IntervalAdd(total, w)
	}
	area, areaBound, err := coilshell.Held(total, "area")
	if err != nil {
		return err
	}
	body.area = Measurement{
		Value:     units.SquareMillimeters(area),
		Exactness: exactnessOf(areaBound),
		Bound:     units.SquareMillimeters(areaBound),
	}

	body.bounds = coilBounds(sh)
	return validateLoftBodyMeasurements(body)
}

// coilBounds is Table CM's Bounds row. Every true point lies within its
// facet's bound of a held triangle, so within δ of the held table's box; the
// box is widened outward by δ to hold the true body. Its sides then sit at
// most δ plus the largest station rounding past the true extremes, since
// every held vertex is within its own rounding of a true point, plus the
// widening's own outward step.
func coilBounds(sh coilshell.Shell) Box {
	lo, hi := sh.Verts[0], sh.Verts[0]
	for _, p := range sh.Verts[1:] {
		lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
		hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
	}
	down := func(x float64) float64 { return math.Nextafter(x-sh.Delta, math.Inf(-1)) }
	up := func(x float64) float64 { return math.Nextafter(x+sh.Delta, math.Inf(1)) }
	minV := r3.NewVec(down(lo.X), down(lo.Y), down(lo.Z))
	maxV := r3.NewVec(up(hi.X), up(hi.Y), up(hi.Z))
	step := 0.0
	for _, c := range []float64{minV.X, minV.Y, minV.Z, maxV.X, maxV.Y, maxV.Z} {
		step = math.Max(step, 2*proofbound.UlpOf(c))
	}
	bound := proofbound.AbsSumUpper(sh.Delta, sh.MaxRound, step)
	return Box{Min: minV, Max: maxV, Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}
}

// buildCoilTopology is Table CB: two planar caps, one Faceted wall per
// recorded profile segment, one rim edge per segment per cap, one helix edge
// per segment junction and one vertex per junction per cap. A whole circle
// has no junction: its wall is a band bounded by its two rim circles, which
// close on one seam vertex per cap. Every loop is stated in the local
// winding and passed through loftLoopCoedges, which carries the shell's one
// orientation decision into the directed boundary.
func buildCoilTopology(ctx context.Context, body *Body, ref producerID, rec coilshell.Record, sh coilshell.Shell, capRegion coil.Iv, walls []coil.Iv) ([]*Face, error) {
	n := rec.N
	segs := rec.Profile.Segments
	vertexAt := func(j int64, v int) *Vertex {
		k := sh.At(j, v)
		return &Vertex{position: sh.Verts[k], bound: units.Millimeters(sh.VertexBound[k])}
	}
	startV := make([]*Vertex, len(segs))
	endV := make([]*Vertex, len(segs))
	for i, seg := range segs {
		startV[i] = vertexAt(0, seg.First)
		endV[i] = vertexAt(n, seg.First)
	}

	rimStart := make([]*Edge, len(segs))
	rimEnd := make([]*Edge, len(segs))
	helix := make([]*Edge, len(segs))
	for i, seg := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		next := rec.Profile.Next(i)
		start, end, err := coilRims(rec, i, startV[i], startV[next], endV[i], endV[next])
		if err != nil {
			return nil, err
		}
		rimStart[i], rimEnd[i] = start, end
		if seg.Closed {
			continue
		}

		// The helix edge is the junction of the walls of the segments into
		// and out of the segment's walk start: a left turn of the recorded
		// profile is convex, the prism's vertical-edge rule, read off the
		// two exact walk tangents there.
		v := seg.First
		inU, inV := coilshell.Tangent(rec, rec.Profile.Prev(i), false)
		outU, outV := coilshell.Tangent(rec, i, true)
		cross := new(big.Rat).Sub(new(big.Rat).Mul(inU, outV), new(big.Rat).Mul(inV, outU))
		hl, ok := coil.HelixLength(rec.Rho[v], rec.Pitch, rec.Turns)
		if !ok {
			return nil, fmt.Errorf(`%w: the coil helix of profile vertex %d has no length`, ErrUnsupported, v)
		}
		hLength, hBound, err := coilshell.Held(rec.Stretched(hl), "helix length")
		if err != nil {
			return nil, err
		}
		chain := 0.0
		for j := int64(0); j <= n; j++ {
			chain = math.Max(chain, sh.VertexBound[sh.At(j, v)])
		}
		helix[i] = &Edge{
			curve: FacetedCurve{Bound: units.Millimeters(chain)},
			start: startV[i], end: endV[i],
			convex: cross.Sign() > 0, length: hLength, lengthBound: hBound,
		}
	}

	capArea, capBound, err := coilshell.Held(rec.Stretched(capRegion), "cap area")
	if err != nil {
		return nil, err
	}
	var capStartLoops, capEndLoops []*Loop
	for loop := range rec.LoopIdx {
		var members []int
		for i, seg := range segs {
			if seg.Loop == loop {
				members = append(members, i)
			}
		}
		m := len(members)
		startCo := make([]coedge, m)
		endCo := make([]coedge, m)
		for k, i := range members {
			startCo[m-1-k] = coedge{edge: rimStart[i], forward: false}
			endCo[k] = coedge{edge: rimEnd[i], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: loop == 0, coedges: loftLoopCoedges(startCo, sh.Reversed)})
		capEndLoops = append(capEndLoops, &Loop{outer: loop == 0, coedges: loftLoopCoedges(endCo, sh.Reversed)})
	}
	capStartSurf, err := planeFromTriangle(sh.Verts, sh.Tris[sh.Walls])
	if err != nil {
		return nil, err
	}
	capEndSurf, err := planeFromTriangle(sh.Verts, sh.Tris[sh.Walls+sh.CapStartCount])
	if err != nil {
		return nil, err
	}
	faces := []*Face{
		{
			surface: capStartSurf, loops: capStartLoops,
			origins: []FeatureRef{{producer: ref, Role: roleCapStart}},
			body:    body, area: capArea, areaBound: capBound,
			axialDelta: sh.Delta, hasAxialDelta: true,
		},
		{
			surface: capEndSurf, loops: capEndLoops,
			origins: []FeatureRef{{producer: ref, Role: roleCapEnd}},
			body:    body, area: capArea, areaBound: capBound,
			axialDelta: sh.Delta, hasAxialDelta: true,
		},
	}
	for i, seg := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		area, areaBound, err := coilshell.Held(walls[i], "wall area")
		if err != nil {
			return nil, err
		}
		// The wall's Faceted bound is the largest β over the vertices its
		// triangles touch: every station of its chords and the next
		// segment's walk start.
		next := rec.Profile.Next(i)
		stations := []int{segs[next].First}
		for c := range seg.Chords {
			stations = append(stations, seg.First+c)
		}
		bound := 0.0
		for j := int64(0); j <= n; j++ {
			for _, v := range stations {
				bound = math.Max(bound, sh.VertexBound[sh.At(j, v)])
			}
		}
		face := &Face{
			surface:   Faceted{Bound: units.Millimeters(bound)},
			origins:   []FeatureRef{{producer: ref, Role: fmt.Sprintf("side(%d,%d)", seg.Loop, seg.Index)}},
			body:      body,
			area:      area,
			areaBound: areaBound,
		}
		if seg.Closed {
			face.loops = []*Loop{
				{outer: true, coedges: loftLoopCoedges([]coedge{{edge: rimStart[i], forward: true}}, sh.Reversed)},
				{outer: true, coedges: loftLoopCoedges([]coedge{{edge: rimEnd[i], forward: false}}, sh.Reversed)},
			}
		} else {
			face.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: rimStart[i], forward: true},
				{edge: helix[next], forward: true},
				{edge: rimEnd[i], forward: false},
				{edge: helix[i], forward: false},
			}, sh.Reversed)}}
		}
		faces = append(faces, face)
	}
	return faces, nil
}

// coilRims builds segment i's two rim edges (Table CB): a Line3 on a line,
// an Arc3 on an arc and a Circle3 on a whole circle, between the cap
// vertices of its walk start and of the next segment's. A circular rim's
// centre is the arc centre's image under the screw motion at θ = 0 and at
// θ = Θ, its axis the section plane's normal there, signed so the rim runs
// counter-clockwise about it from start to end, and its length the arc's
// enclosure widened by L's defect.
func coilRims(rec coilshell.Record, i int, s0, s1, e0, e1 *Vertex) (*Edge, *Edge, error) {
	seg := rec.Profile.Segments[i]
	loopOuter := seg.Loop == 0
	if !seg.IsArc() {
		v, w := seg.First, rec.Profile.Segments[rec.Profile.Next(i)].First
		du := new(big.Rat).Sub(rec.U[w], rec.U[v])
		dv := new(big.Rat).Sub(rec.V[w], rec.V[v])
		l2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		l, ok := proofbound.SqrtFixed(l2)
		if !ok {
			return nil, nil, fmt.Errorf(`%w: the coil rim of profile segment %d has no length`, ErrUnsupported, v)
		}
		length, lengthBound, err := coilshell.Held(rec.Stretched(l), "rim length")
		if err != nil {
			return nil, nil, err
		}
		return &Edge{curve: Line3{}, start: s0, end: s1, convex: loopOuter, length: length, lengthBound: lengthBound},
			&Edge{curve: Line3{}, start: e0, end: e1, convex: loopOuter, length: length, lengthBound: lengthBound}, nil
	}
	length, lengthBound, err := coilshell.Held(rec.Stretched(seg.Length), "rim length")
	if err != nil {
		return nil, nil, err
	}
	radius, _, err := coilshell.Held(seg.Radius, "rim radius")
	if err != nil {
		return nil, nil, err
	}
	cu, cv := coilshell.Centre(seg)
	rims, err := coilshell.RimCircles(rec, cu, cv, seg.Radius, radius)
	if err != nil {
		return nil, nil, err
	}
	centre0, axis0, centre1, axis1 := rims[0].Centre, rims[0].Axis, rims[1].Centre, rims[1].Axis
	// A walk against the circle's counter-clockwise sense, and a placement
	// that reflects, each turn the rim clockwise about the plane normal.
	sign := 1.0
	if seg.Reversed {
		sign = -sign
	}
	if rec.Det.Sign() < 0 {
		sign = -sign
	}
	// A counter-clockwise rim's material lies inside its circle, so the rim
	// is convex; a clockwise one is concave, the prism's rim rule.
	convex := !seg.Reversed
	r := units.Millimeters(radius)
	var c0, c1 Curve
	if seg.Closed {
		c0, c1 = Circle3{Center: centre0, Axis: axis0.Scale(sign), Radius: r}, Circle3{Center: centre1, Axis: axis1.Scale(sign), Radius: r}
	} else {
		c0, c1 = Arc3{Center: centre0, Axis: axis0.Scale(sign), Radius: r}, Arc3{Center: centre1, Axis: axis1.Scale(sign), Radius: r}
	}
	start := &Edge{curve: c0, start: s0, end: s1, convex: convex, length: length, lengthBound: lengthBound,
		curveBound: rims[0].Bound, curveBounded: rims[0].Bounded}
	end := &Edge{curve: c1, start: e0, end: e1, convex: convex, length: length, lengthBound: lengthBound,
		curveBound: rims[1].Bound, curveBounded: rims[1].Bounded}
	return start, end, nil
}
