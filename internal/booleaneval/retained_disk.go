package booleaneval

import (
	"cmp"
	"context"
	"errors"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type diskStation struct {
	sectionrecord.Point2
	angle float64
}

// retainedDiskUnion uses the held stock only to propose angular stations.
// The native disk is certified separately and the ordinary Union checks both operands.
func retainedDiskUnion(ctx context.Context, a, b *topo.Body, meshes OperandMesher, scale float64) (Evaluation, bool, error) {
	if a == nil || b == nil || !(gaptol.Reach(ctx) > 0) {
		return Evaluation{}, false, nil
	}
	stock, disk := a, b
	if !disk.HasStrictPrismBuildSeal() {
		stock, disk = b, a
	}
	held, ok := stock.Payload.(facetedrecord.Payload)
	if !ok || !disk.HasStrictPrismBuildSeal() {
		return Evaluation{}, false, nil
	}
	p, ok := disk.Payload.(prismrecord.Payload)
	if !ok || !stationProfile(p) || len(p.Profile.Outer.Segments) != 1 {
		return Evaluation{}, false, nil
	}
	c, ok := p.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || c.Center != (sectionrecord.Point2{}) || !c.CCW || c.TStart != 0 || c.TEnd != 1 || c.Radius.Unit() != units.Millimeter {
		return Evaluation{}, false, nil
	}
	radius := c.Radius.Base()
	if !(radius > 0) || proofbound.IsNonFinite(radius) {
		return Evaluation{}, false, nil
	}
	tol, d, err := booloperand.PairChord(ctx, a, b)
	if err != nil {
		return Evaluation{}, false, err
	}
	d = chordDiameter(ctx, meshbool.OpUnion, a, b, d)
	chord := operandChord(ctx, disk, tol*scale, d, scale)
	meter := proofbound.NewStrictReplayBudget()
	sealed, err := stationSealed(ctx, disk, p, meter)
	if err != nil || !sealed {
		return Evaluation{}, false, err
	}
	ring, err := proposeDiskRing(ctx, held.Verts, p.Frame.Origin(), radius, chord, meter)
	if err != nil || len(ring) == 0 {
		return Evaluation{}, false, err
	}
	delta, ok, err := diskRingDelta(ctx, ring, radius, meter)
	if err != nil || !ok || delta >= chord {
		return Evaluation{}, false, err
	}
	p.Profile.Outer.Segments = make([]sectionrecord.CurveSegment, len(ring))
	for i, pt := range ring {
		p.Profile.Outer.Segments[i] = sectionrecord.LineSeg{Start: pt, End: ring[(i+1)%len(ring)], TEnd: 1}
	}
	p.Walks = nil
	p.SectionDelta = delta
	dm, err := stationBuild(ctx, disk, p, chord, true, meter)
	if ctx.Err() != nil {
		return Evaluation{}, false, ctx.Err()
	}
	if errors.Is(err, decaderr.ErrDegenerate) || errors.Is(err, decaderr.ErrUnsupported) {
		return Evaluation{}, false, nil
	}
	if dm == nil || err != nil {
		return Evaluation{}, false, err
	}
	alt := &radialInsetMesher{meshes, disk, dm, chord, meter, true}
	if !retainedPlaneSlabDisabled(ctx) && hasFilletTorus(stock) {
		ctx = context.WithValue(ctx, torusSlabTrialKey{}, meter)
	}
	result, err := evaluate(ctx, meshbool.OpUnion, a, b, alt, scale, nil, nil, &operandTwins{}, nil, nil, nil, nil, 0)
	if err != nil {
		return Evaluation{}, false, err
	}
	return result, radialOverlap(a, b, result), nil
}

// Stock points only seed a candidate. The certified radius and finite
// native disk faces come from the sealed disk, independent of the stock.
func proposeDiskRing(ctx context.Context, vertices []r3.Vec, o r3.Vec, radius, chord float64, meter *proofbound.ReplayBudget) ([]sectionrecord.Point2, error) {
	if len(vertices) > 65536 {
		return nil, errStationsInapplicable
	}
	if err := meter.ReserveBundle(ctx, uint64(len(vertices))*3, 1024, 128<<10); err != nil {
		return nil, err
	}

	ulp := func(v float64) float64 { return math.Nextafter(math.Abs(v), math.Inf(1)) - math.Abs(v) }
	near := 16 * max(ulp(o.X), ulp(o.Z), ulp(radius))
	seen := map[sectionrecord.Point2]bool{}
	var samples []diskStation
	for _, v := range vertices {
		if !proofbound.FiniteVec(v) {
			return nil, errStationsInapplicable
		}
		pt := sectionrecord.Point2{U: v.X - o.X, V: v.Z - o.Z}
		if proofbound.IsNonFinite(pt.U) || proofbound.IsNonFinite(pt.V) {
			return nil, errStationsInapplicable
		}
		if math.Abs(math.Hypot(pt.U, pt.V)-radius) > near || seen[pt] {
			continue
		}
		if len(samples) >= 512 {
			return nil, errStationsInapplicable
		}
		seen[pt] = true
		samples = append(samples, diskStation{pt, math.Atan2(pt.V, pt.U)})
	}
	if len(samples) < 4 {
		return nil, errStationsInapplicable
	}
	slices.SortFunc(samples, func(a, b diskStation) int { return cmp.Compare(a.angle, b.angle) })
	// Leave a quarter of the requested tolerance for the inset and rounding.
	step := 2 * math.Acos(1-min(.75*chord/radius, .25))
	if !(step > 0) {
		return nil, errStationsInapplicable
	}
	var ring []sectionrecord.Point2
	for i, s := range samples {
		end := samples[(i+1)%len(samples)].angle
		if i+1 == len(samples) {
			end += 2 * math.Pi
		}
		gap := end - s.angle
		if gap <= near/radius {
			continue
		}
		count := int(math.Ceil(gap / step))
		if count < 1 || count > 1024-len(ring) {
			return nil, errStationsInapplicable
		}
		ring = append(ring, s.Point2)
		for j := 1; j < count; j++ {
			theta := s.angle + gap*float64(j)/float64(count)
			ring = append(ring, sectionrecord.Point2{U: radius * math.Cos(theta), V: radius * math.Sin(theta)})
		}
	}
	// The slight inset separates otherwise coincident held facets. The exact
	// radial bound below charges the entire displacement of the candidate.
	move := min(chord/8, gaptol.Reach(ctx)/100)
	factor := 1 - move/radius
	if !(factor > 0 && factor < 1) {
		return nil, errStationsInapplicable
	}
	for i := range ring {
		ring[i].U *= factor
		ring[i].V *= factor
	}
	return ring, nil
}

// A convex polygon containing the origin admits radial transport to the
// circle on every ray. Its largest radius lies at a vertex. Its smallest
// lies at a closest point on a finite edge. Each convex region lies in the
// other's delta-offset, so the perimeter difference is at most 2π delta.
// The prism builder charges its larger per-walk length and area bounds.
func diskRingDelta(ctx context.Context, ring []sectionrecord.Point2, radius float64, m *proofbound.ReplayBudget) (float64, bool, error) {
	if !(radius > 0) || proofbound.IsNonFinite(radius) || len(ring) < 3 || len(ring) > 1024 {
		return 0, false, nil
	}
	for _, p := range ring {
		if proofbound.IsNonFinite(p.U) || proofbound.IsNonFinite(p.V) {
			return 0, false, nil
		}
	}
	var maxErr float64
	winding := 0
	for i, v := range ring {
		if err := m.ReserveBundle(ctx, 256, 32, 65536); err != nil {
			return 0, false, err
		}
		w := ring[(i+1)%len(ring)]
		next := ring[(i+2)%len(ring)]
		p, q := stationCoords(v), stationCoords(w)
		d := q.minus(p)
		cross := func(a, b stationPoint) int { return stationSub(stationMul(a.x, b.z), stationMul(a.z, b.x)).Sign() }
		if cross(p, q) <= 0 || cross(d, stationCoords(next).minus(q)) <= 0 {
			return 0, false, nil
		}
		if v.V <= 0 && w.V > 0 {
			winding++
		}
		sq := d.radius2()
		dot := stationAdd(stationMul(p.x, d.x), stationMul(p.z, d.z))
		lo, _ := stationOrdered(p.radius2(), q.radius2())
		if dot.Sign() < 0 && stationAdd(dot, sq).Sign() > 0 {
			prod := stationMul(dot, dot)
			lo = stationSub(p.radius2(), prod.Quo(prod, sq))
		}
		maxErr = max(maxErr, math.Nextafter(radius-proofbound.RatSqrtDown(lo), math.Inf(1)), math.Nextafter(proofbound.RatSqrtUp(p.radius2())-radius, math.Inf(1)))
	}
	return maxErr, winding == 1 && !proofbound.IsNonFinite(maxErr), nil
}
