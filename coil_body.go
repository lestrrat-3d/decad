package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file publishes docs/helix-design.md Table CB's topology over the held
// shell coil_build.go builds, and Table CM's four readings from the record's
// closed forms. Interior stations are mesh vertices, never topology: a wall
// is ONE Faceted face per profile segment spanning every turn, bounded by two
// rim edges and two helix edges.

// evalCoil builds the body for one payload record. The first build and
// every placement run it.
func evalCoil(ctx context.Context, d *Document, ref producerID, cp coilPayload) (*Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	rec, err := readCoilRecord(cp)
	if err != nil {
		return nil, err
	}
	sh, err := buildCoilShell(ctx, rec)
	if err != nil {
		return nil, err
	}

	body := &Body{doc: d, origin: FeatureRef{producer: ref, Role: roleBody}, solid: true, kind: BodySolid}
	areaRat := coil.PolygonArea(rec.u, rec.v, rec.loopIdx)
	walls := make([]coil.Iv, len(rec.pts))
	for _, idx := range rec.loopIdx {
		m := len(idx)
		for k := range m {
			v, w := idx[k], idx[(k+1)%m]
			a, ok := coil.SegmentArea(rec.rho[v], rec.zeta[v], rec.rho[w], rec.zeta[w], rec.pitch, rec.turns)
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

	cp.verts, cp.vertexBound, cp.tris, cp.delta = sh.verts, sh.vertexBound, sh.tris, sh.delta
	body.payload = cp
	return body, nil
}

// publishCoilReadings is Table CM: Volume = Θ·Q; the centroid's closed form;
// Area = 2·A_Ω plus every wall's closed form; Bounds over the held table
// widened by δ. Each closed form is an exact enclosure rounded once.
func publishCoilReadings(body *Body, rec coilRecord, sh coilShell, areaRat *big.Rat, walls []coil.Iv) error {
	m := coil.RegionMoments(rec.rho, rec.zeta, rec.loopIdx, areaRat, rec.axis.Side)
	vol, volBound, err := coilHeld(coil.Volume(m, rec.turns), "volume")
	if err != nil {
		return err
	}
	body.volume = Measurement{
		Value:     units.CubicMillimeters(vol),
		Exactness: exactnessOf(volBound),
		Bound:     units.CubicMillimeters(volBound),
	}

	cr, ct, cn, ok := coil.CentroidCoefficients(m, rec.pitch, rec.turns, rec.sigma)
	if !ok {
		return fmt.Errorf(`%w: the coil's first moment is not proven positive`, ErrUnsupported)
	}
	erU, erV := rec.axis.Radial()
	x := proofbound.IntervalAdd(rec.axis.AU, proofbound.IntervalAdd(proofbound.IntervalMul(cr, erU), proofbound.IntervalMul(cn, rec.axis.DU)))
	y := proofbound.IntervalAdd(rec.axis.AV, proofbound.IntervalAdd(proofbound.IntervalMul(cr, erV), proofbound.IntervalMul(cn, rec.axis.DV)))
	z := proofbound.IntervalScale(ct, big.NewRat(int64(rec.axis.Side), 1))
	c := rec.world.point(x, y, z)
	var cv [3]float64
	worst := 0.0
	for axis := range 3 {
		held, bound, err := coilHeld(c[axis], "centroid")
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

	total := proofbound.IntervalScale(coil.Point(areaRat), big.NewRat(2, 1))
	for _, w := range walls {
		total = proofbound.IntervalAdd(total, w)
	}
	area, areaBound, err := coilHeld(total, "area")
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
func coilBounds(sh coilShell) Box {
	lo, hi := sh.verts[0], sh.verts[0]
	for _, p := range sh.verts[1:] {
		lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
		hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
	}
	down := func(x float64) float64 { return math.Nextafter(x-sh.delta, math.Inf(-1)) }
	up := func(x float64) float64 { return math.Nextafter(x+sh.delta, math.Inf(1)) }
	minV := r3.NewVec(down(lo.X), down(lo.Y), down(lo.Z))
	maxV := r3.NewVec(up(hi.X), up(hi.Y), up(hi.Z))
	step := 0.0
	for _, c := range []float64{minV.X, minV.Y, minV.Z, maxV.X, maxV.Y, maxV.Z} {
		step = math.Max(step, 2*proofbound.UlpOf(c))
	}
	bound := proofbound.AbsSumUpper(sh.delta, sh.maxRound, step)
	return Box{Min: minV, Max: maxV, Exactness: exactnessOf(bound), Bound: units.Millimeters(bound)}
}

// buildCoilTopology is Table CB: two planar caps, one Faceted wall per
// profile segment, one rim edge per segment per cap, one helix edge per
// profile vertex and one vertex per profile vertex per cap. Every loop is
// stated in the local winding and passed through loftLoopCoedges, which
// carries the shell's one orientation decision into the directed boundary.
func buildCoilTopology(ctx context.Context, body *Body, ref producerID, rec coilRecord, sh coilShell, areaRat *big.Rat, walls []coil.Iv) ([]*Face, error) {
	stride, n := sh.stride, rec.n
	next := make([]int, stride)
	prev := make([]int, stride)
	isOuter := make([]bool, stride)
	for i, idx := range rec.loopIdx {
		m := len(idx)
		for k, v := range idx {
			next[v] = idx[(k+1)%m]
			prev[v] = idx[(k+m-1)%m]
			isOuter[v] = i == 0
		}
	}

	vertexAt := func(j int64, v int) *Vertex {
		k := sh.at(j, v)
		return &Vertex{position: sh.verts[k], bound: units.Millimeters(sh.vertexBound[k])}
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
		du := new(big.Rat).Sub(rec.u[w], rec.u[v])
		dv := new(big.Rat).Sub(rec.v[w], rec.v[v])
		l2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		l, ok := proofbound.SqrtFixed(l2)
		if !ok {
			return nil, fmt.Errorf(`%w: the coil rim of profile segment %d has no length`, ErrUnsupported, v)
		}
		length, lengthBound, err := coilHeld(l, "rim length")
		if err != nil {
			return nil, err
		}
		rimStart[v] = &Edge{curve: Line3{}, start: startV[v], end: startV[w], convex: isOuter[v], length: length, lengthBound: lengthBound}
		rimEnd[v] = &Edge{curve: Line3{}, start: endV[v], end: endV[w], convex: isOuter[v], length: length, lengthBound: lengthBound}

		// The helix edge is the junction of the walls of the segments into
		// and out of v: a left turn of the recorded profile is convex, the
		// prism's vertical-edge rule.
		p := prev[v]
		inU, inV := new(big.Rat).Sub(rec.u[v], rec.u[p]), new(big.Rat).Sub(rec.v[v], rec.v[p])
		cross := new(big.Rat).Sub(new(big.Rat).Mul(inU, dv), new(big.Rat).Mul(inV, du))
		hl, ok := coil.HelixLength(rec.rho[v], rec.pitch, rec.turns)
		if !ok {
			return nil, fmt.Errorf(`%w: the coil helix of profile vertex %d has no length`, ErrUnsupported, v)
		}
		hLength, hBound, err := coilHeld(hl, "helix length")
		if err != nil {
			return nil, err
		}
		chain := 0.0
		for j := int64(0); j <= n; j++ {
			chain = math.Max(chain, sh.vertexBound[sh.at(j, v)])
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
			wallBound[v] = math.Max(wallBound[v], math.Max(sh.vertexBound[sh.at(j, v)], sh.vertexBound[sh.at(j, w)]))
		}
	}

	capArea, capBound, err := coilRatHeld(areaRat, "cap area")
	if err != nil {
		return nil, err
	}
	var capStartLoops, capEndLoops []*Loop
	for i, idx := range rec.loopIdx {
		m := len(idx)
		startCo := make([]coedge, m)
		endCo := make([]coedge, m)
		for k, v := range idx {
			startCo[m-1-k] = coedge{edge: rimStart[v], forward: false}
			endCo[k] = coedge{edge: rimEnd[v], forward: true}
		}
		capStartLoops = append(capStartLoops, &Loop{outer: i == 0, coedges: loftLoopCoedges(startCo, sh.reversed)})
		capEndLoops = append(capEndLoops, &Loop{outer: i == 0, coedges: loftLoopCoedges(endCo, sh.reversed)})
	}
	capStartSurf, err := planeFromTriangle(sh.verts, sh.tris[sh.walls])
	if err != nil {
		return nil, err
	}
	capEndSurf, err := planeFromTriangle(sh.verts, sh.tris[sh.walls+sh.capStartCount])
	if err != nil {
		return nil, err
	}
	faces := []*Face{
		{
			surface: capStartSurf, loops: capStartLoops,
			origins: []FeatureRef{{producer: ref, Role: roleCapStart}},
			body:    body, area: capArea, areaBound: capBound,
			axialDelta: sh.delta, hasAxialDelta: true,
		},
		{
			surface: capEndSurf, loops: capEndLoops,
			origins: []FeatureRef{{producer: ref, Role: roleCapEnd}},
			body:    body, area: capArea, areaBound: capBound,
			axialDelta: sh.delta, hasAxialDelta: true,
		},
	}
	for i, idx := range rec.loopIdx {
		for k, v := range idx {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			area, areaBound, err := coilHeld(walls[v], "wall area")
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
			}, sh.reversed)}}
			faces = append(faces, face)
		}
	}
	return faces, nil
}
