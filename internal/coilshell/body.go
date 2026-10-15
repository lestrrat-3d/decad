package coilshell

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file publishes docs/helix-design.md Table CB's topology over the held
// shell internal/coilshell builds, and Table CM's four readings from the record's
// closed forms. Interior stations are mesh vertices, never topology: a wall
// is ONE Faceted face per profile segment spanning every turn, bounded by two
// rim edges and two helix edges.

// BuildBody builds the body for one payload record. The first build and
// every placement run it.
func BuildBody(ctx context.Context, env topo.Env, cp Payload) (*topo.Body, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	rec, err := cp.Record()
	if err != nil {
		return nil, err
	}
	sh, err := Build(ctx, rec)
	if err != nil {
		return nil, err
	}
	m, err := coilMoments(ctx, cp, rec)
	if err != nil {
		return nil, err
	}
	walls, err := WallAreas(ctx, rec)
	if err != nil {
		return nil, err
	}

	body := env.Alloc.NewBody()
	body.Solid = true
	body.Origin = provenance.Ref{Producer: env.Ref, Role: provenance.RoleBody}
	body.Kind = topo.BodySolid
	body.Doc = env.Doc
	faces, err := buildCoilTopology(ctx, env, body, rec, sh, m.Area, walls)
	if err != nil {
		return nil, err
	}
	if err := topobuild.AttachFaceLoops(ctx, faces); err != nil {
		return nil, err
	}
	shell := env.Alloc.NewFaceShell()
	shell.Faces = faces
	lump := env.Alloc.NewLump()
	lump.Shells = []*topo.FaceShell{shell}
	body.Lumps = []*topo.Lump{lump}

	if err := publishCoilReadings(body, rec, sh, m, walls); err != nil {
		return nil, err
	}

	cp.Verts, cp.VertexBound, cp.Tris, cp.Delta, cp.MaxRound = sh.Verts, sh.VertexBound, sh.Tris, sh.Delta, sh.MaxRound
	body.Payload = cp
	return body, nil
}

// coilMoments is Table CM's region integrals A_Ω, Q, I and M about the
// axis. A polygon reads its exact shoelace sums over the recorded vertices
// (coil.RegionMoments). A profile with an arc reads the recorded region's
// plane-origin integrals, the ones Revolve reads, each enclosed by its own
// proven bound, re-referenced into the axis frame over intervals.
func coilMoments(ctx context.Context, cp Payload, rec Record) (coil.Moments, error) {
	if !rec.Profile.HasArcs() {
		area := coil.PolygonArea(rec.U, rec.V, rec.LoopIdx)
		return coil.RegionMoments(rec.Rho, rec.Zeta, rec.LoopIdx, area, rec.Axis.Side), nil
	}
	ig, err := cp.Profile.EvaluatorIntegralsContext(ctx, freeform.MomentSecondOrder, freeform.NewFreeformWork())
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
			return coil.Moments{}, fmt.Errorf(`%w: the coil profile's region integrals are not finite`, decaderr.ErrUnsupported)
		}
		*f.dst = iv
	}
	return rec.Axis.Moments(pm), nil
}

// publishCoilReadings is Table CM: Volume = |det L|·Θ·Q; the centroid's
// closed form; Area = 2·A_Ω plus every wall's enclosure, widened by L's
// defect; Bounds over the held table widened by δ. Each closed form is an
// exact enclosure rounded once.
func publishCoilReadings(body *topo.Body, rec Record, sh Shell, m coil.Moments, walls []coil.Iv) error {
	vol, volBound, err := Held(Volume(rec, m), "volume")
	if err != nil {
		return err
	}
	body.Volume = measurement.Measurement{
		Value:     units.CubicMillimeters(vol),
		Exactness: measurement.ExactnessOf(volBound),
		Bound:     units.CubicMillimeters(volBound),
	}

	cr, ct, cn, ok := coil.CentroidCoefficients(m, rec.Pitch, rec.Turns, rec.Sigma)
	if !ok {
		return fmt.Errorf(`%w: the coil's first moment is not proven positive`, decaderr.ErrUnsupported)
	}
	erU, erV := rec.Axis.Radial()
	x := proofbound.IntervalAdd(rec.Axis.AU, proofbound.IntervalAdd(proofbound.IntervalMul(cr, erU), proofbound.IntervalMul(cn, rec.Axis.DU)))
	y := proofbound.IntervalAdd(rec.Axis.AV, proofbound.IntervalAdd(proofbound.IntervalMul(cr, erV), proofbound.IntervalMul(cn, rec.Axis.DV)))
	z := proofbound.IntervalScale(ct, big.NewRat(int64(rec.Axis.Side), 1))
	c := rec.World.Point(x, y, z)
	var cv [3]float64
	worst := 0.0
	for axis := range 3 {
		held, bound, err := Held(c[axis], "centroid")
		if err != nil {
			return err
		}
		cv[axis] = held
		worst = math.Max(worst, bound)
	}
	centroidBound := proofbound.Radius3D(worst)
	body.Centroid = measurement.VecMeasurement{
		Value:     r3.NewVec(cv[0], cv[1], cv[2]),
		Exactness: measurement.ExactnessOf(centroidBound),
		Bound:     units.Millimeters(centroidBound),
	}

	total := rec.Stretched(proofbound.IntervalScale(m.Area, big.NewRat(2, 1)))
	for _, w := range walls {
		total = proofbound.IntervalAdd(total, w)
	}
	area, areaBound, err := Held(total, "area")
	if err != nil {
		return err
	}
	body.Area = measurement.Measurement{
		Value:     units.SquareMillimeters(area),
		Exactness: measurement.ExactnessOf(areaBound),
		Bound:     units.SquareMillimeters(areaBound),
	}

	body.Bounds = coilBounds(sh)
	return topobuild.ValidateLoftBodyMeasurements(body)
}

// coilBounds is Table CM's Bounds row. Every true point lies within its
// facet's bound of a held triangle, so within δ of the held table's box; the
// box is widened outward by δ to hold the true body. Its sides then sit at
// most δ plus the largest station rounding past the true extremes, since
// every held vertex is within its own rounding of a true point, plus the
// widening's own outward step.
func coilBounds(sh Shell) measurement.Box {
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
	return measurement.Box{Min: minV, Max: maxV, Exactness: measurement.ExactnessOf(bound), Bound: units.Millimeters(bound)}
}

// buildCoilTopology is Table CB: two planar caps, one Faceted wall per
// recorded profile segment, one rim edge per segment per cap, one helix edge
// per segment junction and one vertex per junction per cap. A whole circle
// has no junction: its wall is a band bounded by its two rim circles, which
// close on one seam vertex per cap. Every loop is stated in the local
// winding and passed through topobuild.LoftLoopCoedges, which carries the shell's one
// orientation decision into the directed boundary.
func buildCoilTopology(ctx context.Context, env topo.Env, body *topo.Body, rec Record, sh Shell, capRegion coil.Iv, walls []coil.Iv) ([]*topo.Face, error) {
	n := rec.N
	segs := rec.Profile.Segments
	vertexAt := func(j int64, v int) *topo.Vertex {
		k := sh.At(j, v)
		return topobuild.NewVertex(env.Alloc, topo.Vertex{Position: sh.Verts[k], Bound: units.Millimeters(sh.VertexBound[k])})
	}
	startV := make([]*topo.Vertex, len(segs))
	endV := make([]*topo.Vertex, len(segs))
	for i, seg := range segs {
		startV[i] = vertexAt(0, seg.First)
		endV[i] = vertexAt(n, seg.First)
	}

	rimStart := make([]*topo.Edge, len(segs))
	rimEnd := make([]*topo.Edge, len(segs))
	helix := make([]*topo.Edge, len(segs))
	for i, seg := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		next := rec.Profile.Next(i)
		start, end, err := coilRims(env.Alloc, rec, i, startV[i], startV[next], endV[i], endV[next])
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
		inU, inV := Tangent(rec, rec.Profile.Prev(i), false)
		outU, outV := Tangent(rec, i, true)
		cross := new(big.Rat).Sub(new(big.Rat).Mul(inU, outV), new(big.Rat).Mul(inV, outU))
		hl, ok := coil.HelixLength(rec.Rho[v], rec.Pitch, rec.Turns)
		if !ok {
			return nil, fmt.Errorf(`%w: the coil helix of profile vertex %d has no length`, decaderr.ErrUnsupported, v)
		}
		hLength, hBound, err := Held(rec.Stretched(hl), "helix length")
		if err != nil {
			return nil, err
		}
		chain := 0.0
		for j := int64(0); j <= n; j++ {
			chain = math.Max(chain, sh.VertexBound[sh.At(j, v)])
		}
		helix[i] = topobuild.NewEdge(env.Alloc, topo.Edge{
			Convex: cross.Sign() > 0, Length: hLength, LengthBound: hBound, Curve: surfacegeom.FacetedCurve{Bound: units.Millimeters(chain)}, Start: startV[i], End: endV[i],
		})
	}

	capArea, capBound, err := Held(rec.Stretched(capRegion), "cap area")
	if err != nil {
		return nil, err
	}
	var capStartLoops, capEndLoops []*topo.Loop
	for loop := range rec.LoopIdx {
		var members []int
		for i, seg := range segs {
			if seg.Loop == loop {
				members = append(members, i)
			}
		}
		m := len(members)
		startCo := make([]topo.CoEdge, m)
		endCo := make([]topo.CoEdge, m)
		for k, i := range members {
			startCo[m-1-k] = topo.CoEdge{Edge: rimStart[i], Forward: false}
			endCo[k] = topo.CoEdge{Edge: rimEnd[i], Forward: true}
		}
		capStartLoops = append(capStartLoops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: loop == 0, CoEdges: topobuild.LoftLoopCoedges(startCo, sh.Reversed)}))
		capEndLoops = append(capEndLoops, topobuild.NewLoop(env.Alloc, topo.Loop{Outer: loop == 0, CoEdges: topobuild.LoftLoopCoedges(endCo, sh.Reversed)}))
	}
	capStartSurf, err := topobuild.PlaneFromTriangle(sh.Verts, sh.Tris[sh.Walls])
	if err != nil {
		return nil, err
	}
	capEndSurf, err := topobuild.PlaneFromTriangle(sh.Verts, sh.Tris[sh.Walls+sh.CapStartCount])
	if err != nil {
		return nil, err
	}
	faces := []*topo.Face{
		topobuild.NewFace(env.Alloc, topo.Face{
			Area:          capArea,
			AreaBound:     capBound,
			AxialDelta:    sh.Delta,
			HasAxialDelta: true,
			Surface:       capStartSurf,
			Origins:       []provenance.Ref{{Producer: env.Ref, Role: provenance.RoleCapStart}},
			Body:          body,
			Loops:         capStartLoops,
		}),
		topobuild.NewFace(env.Alloc, topo.Face{
			Area:          capArea,
			AreaBound:     capBound,
			AxialDelta:    sh.Delta,
			HasAxialDelta: true,
			Surface:       capEndSurf,
			Origins:       []provenance.Ref{{Producer: env.Ref, Role: provenance.RoleCapEnd}},
			Body:          body,
			Loops:         capEndLoops,
		}),
	}
	for i, seg := range segs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		area, areaBound, err := Held(walls[i], "wall area")
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
		face := topobuild.NewFace(env.Alloc, topo.Face{
			Area:      area,
			AreaBound: areaBound,
			Surface:   surfacegeom.Faceted{Bound: units.Millimeters(bound)},
			Origins:   []provenance.Ref{{Producer: env.Ref, Role: fmt.Sprintf("side(%d,%d)", seg.Loop, seg.Index)}},
			Body:      body,
		})
		if seg.Closed {
			face.Loops = []*topo.Loop{
				topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: topobuild.LoftLoopCoedges([]topo.CoEdge{{Edge: rimStart[i], Forward: true}}, sh.Reversed)}),
				topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: topobuild.LoftLoopCoedges([]topo.CoEdge{{Edge: rimEnd[i], Forward: false}}, sh.Reversed)}),
			}
		} else {
			face.Loops = []*topo.Loop{topobuild.NewLoop(env.Alloc, topo.Loop{Outer: true, CoEdges: topobuild.LoftLoopCoedges([]topo.CoEdge{
				{Edge: rimStart[i], Forward: true},
				{Edge: helix[next], Forward: true},
				{Edge: rimEnd[i], Forward: false},
				{Edge: helix[i], Forward: false},
			}, sh.Reversed),
			})}
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
func coilRims(alloc topo.Alloc, rec Record, i int, s0, s1, e0, e1 *topo.Vertex) (*topo.Edge, *topo.Edge, error) {
	seg := rec.Profile.Segments[i]
	loopOuter := seg.Loop == 0
	if !seg.IsArc() {
		v, w := seg.First, rec.Profile.Segments[rec.Profile.Next(i)].First
		du := new(big.Rat).Sub(rec.U[w], rec.U[v])
		dv := new(big.Rat).Sub(rec.V[w], rec.V[v])
		l2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		l, ok := proofbound.SqrtFixed(l2)
		if !ok {
			return nil, nil, fmt.Errorf(`%w: the coil rim of profile segment %d has no length`, decaderr.ErrUnsupported, v)
		}
		length, lengthBound, err := Held(rec.Stretched(l), "rim length")
		if err != nil {
			return nil, nil, err
		}
		return topobuild.NewEdge(alloc, topo.Edge{Convex: loopOuter, Length: length, LengthBound: lengthBound, Curve: surfacegeom.Line3{}, Start: s0, End: s1}),
			topobuild.NewEdge(alloc, topo.Edge{Convex: loopOuter, Length: length, LengthBound: lengthBound, Curve: surfacegeom.Line3{}, Start: e0, End: e1}), nil
	}
	length, lengthBound, err := Held(rec.Stretched(seg.Length), "rim length")
	if err != nil {
		return nil, nil, err
	}
	radius, _, err := Held(seg.Radius, "rim radius")
	if err != nil {
		return nil, nil, err
	}
	cu, cv := Centre(seg)
	rims, err := RimCircles(rec, cu, cv, seg.Radius, radius)
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
	var c0, c1 surfacegeom.Curve
	if seg.Closed {
		c0, c1 = surfacegeom.Circle3{Center: centre0, Axis: axis0.Scale(sign), Radius: r}, surfacegeom.Circle3{Center: centre1, Axis: axis1.Scale(sign), Radius: r}
	} else {
		c0, c1 = surfacegeom.Arc3{Center: centre0, Axis: axis0.Scale(sign), Radius: r}, surfacegeom.Arc3{Center: centre1, Axis: axis1.Scale(sign), Radius: r}
	}
	start := topobuild.NewEdge(alloc, topo.Edge{
		Convex:       convex,
		Length:       length,
		LengthBound:  lengthBound,
		CurveBound:   rims[0].Bound,
		CurveBounded: rims[0].Bounded,
		Curve:        c0,
		Start:        s0,
		End:          s1,
	})
	end := topobuild.NewEdge(alloc, topo.Edge{
		Convex:       convex,
		Length:       length,
		LengthBound:  lengthBound,
		CurveBound:   rims[1].Bound,
		CurveBounded: rims[1].Bounded,
		Curve:        c1,
		Start:        e0,
		End:          e1,
	})
	return start, end, nil
}
