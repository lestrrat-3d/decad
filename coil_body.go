package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/coilshell"
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
			AU: cp.line.aU, AV: cp.line.aV, AUBound: cp.line.aUBound, AVBound: cp.line.aVBound,
			DU: cp.line.dU, DV: cp.line.dV, DUBound: cp.line.dUBound, DVBound: cp.line.dVBound,
			Side: cp.side,
		},
		Pitch: cp.pitch, Turns: cp.turns, LeftHand: cp.leftHand,
		StationsPerTurn: coilStationsPerTurn, MaxStations: maxCoilStations,
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

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	areaRat := coil.PolygonArea(rec.U, rec.V, rec.LoopIdx)
	walls := make([]coil.Iv, len(rec.Pts))
	for _, idx := range rec.LoopIdx {
		m := len(idx)
		for k := range m {
			v, w := idx[k], idx[(k+1)%m]
			a, ok := coil.SegmentArea(rec.Rho[v], rec.Zeta[v], rec.Rho[w], rec.Zeta[w], rec.Pitch, rec.Turns)
			if !ok {
				return nil, fmt.Errorf(`%w: the coil wall of profile segment %d has no area enclosure`, ErrUnsupported, v)
			}
			walls[v] = a
		}
	}
	faces, err := buildCoilTopology(ctx, body, ref, rec, sh, areaRat, walls)
	if err != nil {
		return nil, err
	}
	if err := attachFaceLoopsContext(ctx, faces); err != nil {
		return nil, err
	}
	body.lumps = []*Lump{{shells: []*Shell{{faces: faces}}}}

	if err := publishCoilReadings(body, rec, sh, areaRat, walls); err != nil {
		return nil, err
	}

	cp.verts, cp.vertexBound, cp.tris, cp.delta, cp.maxRound = sh.Verts, sh.VertexBound, sh.Tris, sh.Delta, sh.MaxRound
	body.payload = cp
	return body, nil
}

// publishCoilReadings is Table CM: Volume = |det L|·Θ·Q; the centroid's
// closed form; Area = 2·A_Ω plus every wall's closed form, widened by L's
// defect; Bounds over the held table widened by δ. Each closed form is an
// exact enclosure rounded once.
func publishCoilReadings(body *Body, rec coilshell.Record, sh coilshell.Shell, areaRat *big.Rat, walls []coil.Iv) error {
	m := coil.RegionMoments(rec.Rho, rec.Zeta, rec.LoopIdx, areaRat, rec.Axis.Side)
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

	total := rec.Stretched(proofbound.IntervalScale(coil.Point(areaRat), big.NewRat(2, 1)))
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
// profile segment, one rim edge per segment per cap, one helix edge per
// profile vertex and one vertex per profile vertex per cap. Every loop is
// stated in the local winding and passed through loftLoopCoedges, which
// carries the shell's one orientation decision into the directed boundary.
func buildCoilTopology(ctx context.Context, body *Body, ref producerID, rec coilshell.Record, sh coilshell.Shell, areaRat *big.Rat, walls []coil.Iv) ([]*Face, error) {
	stride, n := sh.Stride, rec.N
	next := make([]int, stride)
	prev := make([]int, stride)
	isOuter := make([]bool, stride)
	for i, idx := range rec.LoopIdx {
		m := len(idx)
		for k, v := range idx {
			next[v] = idx[(k+1)%m]
			prev[v] = idx[(k+m-1)%m]
			isOuter[v] = i == 0
		}
	}

	vertexAt := func(j int64, v int) *Vertex {
		k := sh.At(j, v)
		return &Vertex{position: sh.Verts[k], bound: units.Millimeters(sh.VertexBound[k])}
	}
	startV := make([]*Vertex, stride)
	endV := make([]*Vertex, stride)
	for v := range stride {
		startV[v] = vertexAt(0, v)
		endV[v] = vertexAt(n, v)
	}

	rimStart := make([]*Edge, stride)
	rimEnd := make([]*Edge, stride)
	helix := make([]*Edge, stride)
	for v := range stride {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		w := next[v]
		du := new(big.Rat).Sub(rec.U[w], rec.U[v])
		dv := new(big.Rat).Sub(rec.V[w], rec.V[v])
		l2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		l, ok := proofbound.SqrtFixed(l2)
		if !ok {
			return nil, fmt.Errorf(`%w: the coil rim of profile segment %d has no length`, ErrUnsupported, v)
		}
		length, lengthBound, err := coilshell.Held(rec.Stretched(l), "rim length")
		if err != nil {
			return nil, err
		}
		rimStart[v] = &Edge{curve: Line3{}, start: startV[v], end: startV[w], convex: isOuter[v], length: length, lengthBound: lengthBound}
		rimEnd[v] = &Edge{curve: Line3{}, start: endV[v], end: endV[w], convex: isOuter[v], length: length, lengthBound: lengthBound}

		// The helix edge is the junction of the walls of the segments into
		// and out of v: a left turn of the recorded profile is convex, the
		// prism's vertical-edge rule.
		p := prev[v]
		inU, inV := new(big.Rat).Sub(rec.U[v], rec.U[p]), new(big.Rat).Sub(rec.V[v], rec.V[p])
		cross := new(big.Rat).Sub(new(big.Rat).Mul(inU, dv), new(big.Rat).Mul(inV, du))
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
		helix[v] = &Edge{
			curve: FacetedCurve{Bound: units.Millimeters(chain)},
			start: startV[v], end: endV[v],
			convex: cross.Sign() > 0, length: hLength, lengthBound: hBound,
		}
	}

	// Each wall's Faceted bound is the largest β over the vertices its
	// triangles touch: the stations of its segment's two ends.
	wallBound := make([]float64, stride)
	for v := range stride {
		w := next[v]
		for j := int64(0); j <= n; j++ {
			wallBound[v] = math.Max(wallBound[v], math.Max(sh.VertexBound[sh.At(j, v)], sh.VertexBound[sh.At(j, w)]))
		}
	}

	capArea, capBound, err := coilshell.Held(rec.Stretched(coil.Point(areaRat)), "cap area")
	if err != nil {
		return nil, err
	}
	var capStartLoops, capEndLoops []*Loop
	for i, idx := range rec.LoopIdx {
		m := len(idx)
		startCo := make([]coedge, m)
		endCo := make([]coedge, m)
		for k, v := range idx {
			startCo[m-1-k] = coedge{edge: rimStart[v], forward: false}
			endCo[k] = coedge{edge: rimEnd[v], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: i == 0, coedges: loftLoopCoedges(startCo, sh.Reversed)})
		capEndLoops = append(capEndLoops, &Loop{outer: i == 0, coedges: loftLoopCoedges(endCo, sh.Reversed)})
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
	for i, idx := range rec.LoopIdx {
		for k, v := range idx {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			area, areaBound, err := coilshell.Held(walls[v], "wall area")
			if err != nil {
				return nil, err
			}
			face := &Face{
				surface:   Faceted{Bound: units.Millimeters(wallBound[v])},
				origins:   []FeatureRef{{producer: ref, Role: fmt.Sprintf("side(%d,%d)", i, k)}},
				body:      body,
				area:      area,
				areaBound: areaBound,
			}
			face.loops = []*Loop{{outer: true, coedges: loftLoopCoedges([]coedge{
				{edge: rimStart[v], forward: true},
				{edge: helix[next[v]], forward: true},
				{edge: rimEnd[v], forward: false},
				{edge: helix[v], forward: false},
			}, sh.Reversed)}}
			faces = append(faces, face)
		}
	}
	return faces, nil
}
