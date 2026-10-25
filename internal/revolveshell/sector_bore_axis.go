package revolveshell

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sectorBoreAxis is how the annular bore Cut reads the join's axis
// (agent-docs/revolve-axis-design.md §9). xBoreAxis is the +X reading, and
// generalBoreAxis the reading of a certificate from the general record branch.
type sectorBoreAxis interface {
	// ring returns the bore's low and high inner ring vertices at the joined
	// ring's stations, and a bound on how far the ring and its chords sit
	// from the bore wall's true circles on the two caps.
	ring(ctx context.Context, caps SectorBoreCaps, radius float64, angles []float64,
		maxStep float64) ([]r3.Vec, []r3.Vec, float64, error)
	// spanClear reports whether every point of the revolved section span
	// lies farther than clearance from the axis.
	spanClear(span facetedrecord.RevolveSectionSpan, clearance float64) bool
	// triangleClear reports whether every point of the triangle lies
	// farther than clearance from the axis.
	triangleClear(tri [3]int, verts []r3.Vec, clearance float64) bool
	// patchAxes names the coordinate plane the audit patches project onto.
	patchAxes() [2]int
	// height bounds the axial distance between the two caps from above.
	height(caps SectorBoreCaps) float64
}

// xBoreAxis reads a +X certificate: caps at x = LowX and x = HighX, radii in
// the YZ plane.
type xBoreAxis struct{}

func (xBoreAxis) ring(ctx context.Context, caps SectorBoreCaps, radius float64, angles []float64,
	maxStep float64) ([]r3.Vec, []r3.Vec, float64, error) {
	inner, bound, err := circularbounds.BoreRingAtStep(ctx, radius, angles, maxStep)
	if err != nil {
		return nil, nil, 0, err
	}
	low, high := make([]r3.Vec, len(inner)), make([]r3.Vec, len(inner))
	for i, p := range inner {
		low[i] = r3.Vec{X: caps.LowX, Y: p.U, Z: p.V}
		high[i] = r3.Vec{X: caps.HighX, Y: p.U, Z: p.V}
	}
	return low, high, bound, nil
}

func (xBoreAxis) spanClear(span facetedrecord.RevolveSectionSpan, clearance float64) bool {
	return min(span.Start.V, span.End.V) > clearance
}

func (xBoreAxis) triangleClear(tri [3]int, verts []r3.Vec, clearance float64) bool {
	return RevolveSectorTriangleClear(tri, verts, clearance)
}

func (xBoreAxis) patchAxes() [2]int { return [2]int{1, 2} }

func (xBoreAxis) height(caps SectorBoreCaps) float64 {
	return proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(caps.HighX), proofarith.FloatRat(caps.LowX)))
}

// generalBoreAxis reads a certificate from the general record branch about
// the exact axis A (agent-docs/revolve-axis-design.md §9.2, §9.3). The caps
// stand at the exact A-scaled axial coordinates lowZ and highZ, the ring is
// placed with the blank Revolve's own lift, and deviation is the bound on how
// far the tool's wall sits from the coaxial cylinder of its radius.
type generalBoreAxis struct {
	axis                coneaxis.Axis
	frame               r3.Frame
	lift                ringLiftAbout
	lowPoint, highPoint sectionrecord.Point2
	lowZ, highZ         *big.Rat
	deviation           float64
}

// ring places each inner vertex with the blank Revolve's own lift at the
// cap's axial coordinate, the bore radius and the station angle, and charges
// its exact SweptPointGap against the exact cap point at that radius, swept
// about the record's axis (§9.3). The bound adds the chord sag at the
// largest station step and the tool's deviation.
func (g generalBoreAxis) ring(ctx context.Context, _ SectorBoreCaps, radius float64, angles []float64,
	maxStep float64) ([]r3.Vec, []r3.Vec, float64, error) {
	if radius <= 0 || proofbound.IsNonFinite(radius) || len(angles) < 3 {
		return nil, nil, 0, fmt.Errorf("%w: circular bore ring has no positive radius or complete stations",
			decaderr.ErrUnsupported)
	}
	type capRing struct {
		point  sectionrecord.Point2
		z      *big.Rat
		axial  float64
		plane  sectionrecord.Point2
		within proofbound.WalkEndBound
	}
	caps := [2]capRing{{point: g.lowPoint, z: g.lowZ}, {point: g.highPoint, z: g.highZ}}
	for i := range caps {
		caps[i].axial, _ = g.lift.payload.Ax.ToAxis(caps[i].point.U, caps[i].point.V)
		plane, within, ok := g.borePlanePoint(caps[i].z, radius)
		if !ok {
			return nil, nil, 0, fmt.Errorf("%w: bore circle has no plane point", decaderr.ErrUnsupported)
		}
		caps[i].plane, caps[i].within = plane, within
	}
	low, high := make([]r3.Vec, len(angles)), make([]r3.Vec, len(angles))
	maxGap := 0.0
	for j, angle := range angles {
		if err := ctx.Err(); err != nil {
			return nil, nil, 0, err
		}
		sin, cos, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
		if !ok {
			return nil, nil, 0, fmt.Errorf("%w: circular bore ring angle has no trigonometric enclosure",
				decaderr.ErrUnsupported)
		}
		for k, out := range [2][]r3.Vec{low, high} {
			c := caps[k]
			held := g.lift.lift.Point(g.lift.basis, r3.Identity(), c.axial, radius, angle)
			gap := g.lift.lift.SweptPointGap(g.lift.bound, r3.Identity(), c.plane.U, c.plane.V, c.within,
				sin, cos, held)
			if !proofbound.FiniteVec(held) || proofbound.IsNonFinite(gap) {
				return nil, nil, 0, fmt.Errorf("%w: bore ring vertex is non-finite", decaderr.ErrUnsupported)
			}
			out[j] = held
			maxGap = max(maxGap, gap)
		}
	}
	sag := proofbound.ProductUpper(radius, proofbound.ProductUpper(maxStep, maxStep)/8)
	return low, high, proofbound.AbsSumUpper(sag, maxGap, g.deviation), nil
}

// borePlanePoint encloses the exact plane-local point at A-scaled axial
// coordinate z and the given radius in millimetres on the positive radial side:
// q = (z/A·A)·A_p + (radius/|A|)·N_p, where A_p is A read in the sketch
// plane and N_p its left normal, so that Z(q) = z and R(q) = radius·|A|. It
// returns the nearest float point and a per-coordinate bound whose box holds
// q, reading 1/|A| through the norm bracket.
func (g generalBoreAxis) borePlanePoint(z *big.Rat, radius float64) (sectionrecord.Point2,
	proofbound.WalkEndBound, bool) {
	a := g.axis.Vec()
	au, av := proofarith.FloatRat(a.Dot(g.frame.U())), proofarith.FloatRat(a.Dot(g.frame.V()))
	normSq := new(big.Rat).Add(new(big.Rat).Mul(au, au), new(big.Rat).Mul(av, av))
	lo, hi := g.axis.NormBracket()
	if normSq.Sign() <= 0 || !(lo > 0) || proofbound.IsNonFinite(hi) {
		return sectionrecord.Point2{}, proofbound.WalkEndBound{}, false
	}
	scale := new(big.Rat).Quo(z, normSq)
	r := proofarith.FloatRat(radius)
	var held [2]float64
	var within [2]float64
	for i, pair := range [2][2]*big.Rat{{au, new(big.Rat).Neg(av)}, {av, au}} {
		base := new(big.Rat).Mul(scale, pair[0])
		offset := new(big.Rat).Mul(r, pair[1])
		x := new(big.Rat).Add(base, new(big.Rat).Quo(offset, proofarith.FloatRat(hi)))
		y := new(big.Rat).Add(base, new(big.Rat).Quo(offset, proofarith.FloatRat(lo)))
		held[i], _ = new(big.Rat).Quo(new(big.Rat).Add(x, y), big.NewRat(2, 1)).Float64()
		h := proofarith.FloatRat(held[i])
		dx, dy := new(big.Rat).Sub(x, h), new(big.Rat).Sub(y, h)
		within[i] = max(proofbound.RatFloatUp(dx.Abs(dx)), proofbound.RatFloatUp(dy.Abs(dy)))
		if proofbound.IsNonFinite(held[i]) || proofbound.IsNonFinite(within[i]) {
			return sectionrecord.Point2{}, proofbound.WalkEndBound{}, false
		}
	}
	return sectionrecord.Point2{U: held[0], V: held[1]}, proofbound.WalkEndBound{U: within[0], V: within[1]}, true
}

// spanClear reads the span's two ends' exact plane-scaled R as millimetres
// rounded down. R is linear along a straight span, so its ends bound it.
func (g generalBoreAxis) spanClear(span facetedrecord.RevolveSectionSpan, clearance float64) bool {
	for _, p := range [2]sectionrecord.Point2{span.Start, span.End} {
		_, r := planeScaledRat(g.axis, g.frame, p)
		radius, ok := g.axis.ScaledDown(r)
		if !ok || radius <= clearance {
			return false
		}
	}
	return true
}

func (g generalBoreAxis) triangleClear(tri [3]int, verts []r3.Vec, clearance float64) bool {
	return g.axis.TriangleBeyond([3]r3.Vec{verts[tri[0]], verts[tri[1]], verts[tri[2]]}, clearance)
}

func (g generalBoreAxis) patchAxes() [2]int { return capPatchAxes(g.axis) }

func (g generalBoreAxis) height(SectorBoreCaps) float64 {
	height, ok := g.axis.ScaledUp(new(big.Rat).Sub(g.highZ, g.lowZ))
	if !ok {
		return 0
	}
	return height
}

// revolveSectorBoreCapsAbout is RevolveSectorBoreCapRecord for a certificate
// from the general record branch (agent-docs/revolve-axis-design.md §9.2).
// It reads every coordinate as the exact plane-scaled (Z, R): the axis span
// has R = 0 at both ends, each cap has one exact Z at both ends, and the low
// cap's Z is below the high cap's. LowX and HighX stay zero; the caps' radii
// are their R/|A| rounded down. Each axis vertex must equal the ring's own
// lift of its point.
func revolveSectorBoreCapsAbout(joined facetedrecord.Payload) (SectorBoreCaps, generalBoreAxis, bool) {
	var caps SectorBoreCaps
	var none generalBoreAxis
	record := joined.RevolveLoftJoin
	if record == nil || record.Certificate == nil || record.Blank == nil ||
		record.CapRing == nil || record.Certificate.Section.Blank != record.Blank ||
		record.Certificate.Section.General == nil ||
		record.BoundaryBound != joined.MeshBound ||
		record.VolumeBound != joined.VolSymDiff || record.AreaBound != joined.AreaSlack ||
		joined.Xform != r3.Identity() || !joined.ContactAudited {
		return caps, none, false
	}
	section, ring := record.Certificate.Section, record.CapRing
	spans := section.Spans
	count, n := len(spans), len(ring.Angles)
	if count < 4 || len(ring.Rings) != count || len(ring.Groups) != count ||
		section.AxisSpan < 0 || section.AxisSpan >= count ||
		n < 16 || n > revolvemesh.SectorBoreMaxStations ||
		len(joined.Tris) != len(joined.Src) ||
		len(joined.Verts) != len(joined.VertexBound) {
		return caps, none, false
	}
	lift, ok := newRingLiftAbout(section)
	if !ok {
		return caps, none, false
	}
	axisSpan := section.AxisSpan
	caps.Low, caps.High = (axisSpan+count-1)%count, (axisSpan+1)%count
	low, high, along := spans[caps.Low], spans[caps.High], spans[axisSpan]
	read := func(p sectionrecord.Point2) (*big.Rat, *big.Rat) {
		return planeScaledRat(section.Axis, section.Frame, p)
	}
	_, alongStartR := read(along.Start)
	_, alongEndR := read(along.End)
	lowZ, lowR := read(low.Start)
	lowEndZ, _ := read(low.End)
	highZ, _ := read(high.Start)
	highEndZ, highR := read(high.End)
	if alongStartR.Sign() != 0 || alongEndR.Sign() != 0 ||
		low.End != along.Start || high.Start != along.End ||
		lowR.Sign() <= 0 || highR.Sign() <= 0 ||
		lowZ.Cmp(lowEndZ) != 0 || highZ.Cmp(highEndZ) != 0 || lowZ.Cmp(highZ) >= 0 ||
		low.Face == nil || high.Face == nil ||
		len(low.Origins) == 0 || len(high.Origins) == 0 {
		return caps, none, false
	}
	if _, ok := low.Face.Surface.(surfacegeom.Plane); !ok {
		return caps, none, false
	}
	if _, ok := high.Face.Surface.(surfacegeom.Plane); !ok {
		return caps, none, false
	}
	lowRadius, lowOK := section.Axis.ScaledDown(lowR)
	highRadius, highOK := section.Axis.ScaledDown(highR)
	if !lowOK || !highOK || lowRadius <= 0 || highRadius <= 0 {
		return caps, none, false
	}
	caps.LowRadius, caps.HighRadius = lowRadius, highRadius
	caps.LowGroup, caps.HighGroup = ring.Groups[caps.Low], ring.Groups[caps.High]
	if caps.LowGroup < 0 || caps.HighGroup < 0 ||
		caps.LowGroup >= len(joined.Groups) || caps.HighGroup >= len(joined.Groups) ||
		caps.LowGroup == caps.HighGroup ||
		len(ring.Rings[axisSpan]) != 1 || len(ring.Rings[caps.High]) != 1 ||
		len(ring.Rings[caps.Low]) != n ||
		len(ring.Rings[(caps.High+1)%count]) != n {
		return caps, none, false
	}
	caps.LowAxis, caps.HighAxis = ring.Vertex(axisSpan, 0), ring.Vertex(caps.High, 0)
	caps.LowOuter = ring.Rings[caps.Low]
	caps.HighOuter = ring.Rings[(caps.High+1)%count]
	if caps.LowAxis == caps.HighAxis ||
		caps.LowAxis < 0 || caps.HighAxis < 0 ||
		caps.LowAxis >= len(joined.Verts) || caps.HighAxis >= len(joined.Verts) {
		return caps, none, false
	}
	var zero proofbound.RatInterval
	lowAxis, _ := lift.vertex(along.Start, true, 0, zero, zero)
	highAxis, _ := lift.vertex(along.End, true, 0, zero, zero)
	if joined.Verts[caps.LowAxis] != lowAxis || joined.Verts[caps.HighAxis] != highAxis {
		return caps, none, false
	}
	return caps, generalBoreAxis{axis: section.Axis, frame: section.Frame, lift: lift,
		lowPoint: low.Start, highPoint: high.End, lowZ: lowZ, highZ: highZ}, true
}

// sectorCylinderCutAbout is SectorCylinderCut for a join whose certificate
// came from the general record branch (agent-docs/revolve-axis-design.md §9).
// The tool must be an unplaced prism of one whole circle centred on its frame
// origin, with its normal N on the axis (N·A > 0) and its origin on the axis
// line, each within massmoment.DriftSinLimit; a tool outside that keeps
// ordinary dispatch. Both of the tool's end discs must lie strictly outside
// the caps. The bore radius must lie strictly inside both cap radii; a bore
// through a section band refuses.
func sectorCylinderCutAbout(ctx context.Context, env topo.Env, joined facetedrecord.Payload,
	tool *topo.Body) (*topo.Body, bool, error) {
	caps, bore, ok := revolveSectorBoreCapsAbout(joined)
	if !ok {
		return nil, true, fmt.Errorf("%w: joined section has no complete planar cap-ring record", decaderr.ErrUnsupported)
	}
	prism, ok := tool.Payload.(prismrecord.Payload)
	if !ok || prism.SurfaceResult || prism.Xform != r3.Identity() ||
		prism.SectionDelta != 0 || prism.Z0Delta != 0 || prism.Z1Delta != 0 ||
		len(prism.Profile.Holes) != 0 || len(prism.Profile.Outer.Segments) != 1 {
		return nil, false, nil
	}
	circle, ok := prism.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || circle.Center != (sectionrecord.Point2{}) ||
		!((circle.TStart == 0 && circle.TEnd == 1) ||
			(circle.TStart == 1 && circle.TEnd == 0)) {
		return nil, false, nil
	}
	if !coaxialBoreFrame(prism.Frame, bore.axis) {
		return nil, false, nil
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil {
		return nil, true, err
	}
	if !proofbound.FiniteVec(r3.Vec{X: prism.Z0, Y: prism.Z1, Z: radius}) || radius <= 0 {
		return nil, true, fmt.Errorf("%w: bore has no finite positive radius", decaderr.ErrUnsupported)
	}
	if !boreToolSpansCaps(prism, bore.axis, radius, bore.lowZ, bore.highZ) {
		return nil, true, fmt.Errorf("%w: cylinder does not span both section caps", decaderr.ErrUnsupported)
	}
	deviation, ok := boreToolDeviation(prism, bore.axis, radius)
	if !ok {
		return nil, true, fmt.Errorf("%w: cylinder has no bounded distance from the axis", decaderr.ErrUnsupported)
	}
	bore.deviation = deviation
	wallGroup, err := sectorBoreWall(tool)
	if err != nil {
		return nil, true, err
	}
	if radius < min(caps.LowRadius, caps.HighRadius) {
		result, err := buildSectorAnnularCut(ctx, joined, caps, radius, wallGroup, bore)
		if err != nil {
			return nil, true, err
		}
		body, err := facetedrecord.Build(ctx, env, result)
		return body, true, err
	}
	if radius > caps.LowRadius && radius < caps.HighRadius {
		return nil, true, fmt.Errorf("%w: bore through a section band about a general axis", decaderr.ErrUnsupported)
	}
	return nil, true, fmt.Errorf("%w: bore has no certified section crossing", decaderr.ErrUnsupported)
}

// coaxialBoreFrame admits a tool frame whose normal N and origin O lie on
// the axis to rounding (agent-docs/revolve-axis-design.md §9.1): N·A > 0,
// |N×A|² ≤ L²·|N|²·(A·A) and |O×A|² ≤ L²·|O|²·(A·A), with L =
// massmoment.DriftSinLimit, each an exact comparison of the held values. The
// limit declines a deliberately tilted or offset tool; boreToolDeviation
// charges what passes.
func coaxialBoreFrame(frame r3.Frame, axis coneaxis.Axis) bool {
	a, n, o := axis.Vec(), frame.N(), frame.Origin()
	if !proofbound.FiniteVec(n) || !proofbound.FiniteVec(o) {
		return false
	}
	da, dn := proofarith.DyVec(a), proofarith.DyVec(n)
	if proofarith.DvDot(dn, da).Sign() <= 0 {
		return false
	}
	limit := proofarith.MustDyOf(massmoment.DriftSinLimit)
	limitSq := proofarith.DyMul(proofarith.DyMul(limit, limit), axis.NormSq())
	onAxis := func(v proofarith.DyV3) bool {
		cross := proofarith.DvCross(v, da)
		return proofarith.DyCmp(proofarith.DvDot(cross, cross),
			proofarith.DyMul(limitSq, proofarith.DvDot(v, v))) <= 0
	}
	return onAxis(dn) && onAxis(proofarith.DyVec(o))
}

// boreToolSpansCaps reports whether both of the tool's end discs lie
// strictly outside the slab between the caps. A point of the disc at level z
// is O + z·N + a·U + b·V with a² + b² ≤ r², so its axial coordinate is at
// most (O + z·N)·A + r·√((U·A)² + (V·A)²); the test compares squares
// exactly.
func boreToolSpansCaps(prism prismrecord.Payload, axis coneaxis.Axis, radius float64, lowZ, highZ *big.Rat) bool {
	frame := prism.Frame
	a := ratVec(axis.Vec())
	o, n := ratVec(frame.Origin()), ratVec(frame.N())
	ua, va := ratDot(ratVec(frame.U()), a), ratDot(ratVec(frame.V()), a)
	r := proofarith.FloatRat(radius)
	tilt := new(big.Rat).Mul(new(big.Rat).Mul(r, r),
		new(big.Rat).Add(new(big.Rat).Mul(ua, ua), new(big.Rat).Mul(va, va)))
	level := func(z float64) *big.Rat {
		return new(big.Rat).Add(ratDot(o, a), new(big.Rat).Mul(proofarith.FloatRat(z), ratDot(n, a)))
	}
	beyond := func(gap *big.Rat) bool {
		return gap.Sign() > 0 && new(big.Rat).Mul(gap, gap).Cmp(tilt) > 0
	}
	return beyond(new(big.Rat).Sub(lowZ, level(prism.Z0))) && beyond(new(big.Rat).Sub(level(prism.Z1), highZ))
}

// boreToolDeviation bounds how far the tool's side wall lies from the
// coaxial cylinder of radius r about A (agent-docs/revolve-axis-design.md
// §9.4). A wall point is p = O + z·N + r·w with w = cos θ·U + sin θ·V and z
// in [Z0, Z1], and its distance from the axis line is |p×A|/|A|. Then
//
//	| |p×A|/|A| − r | ≤ (|O×A| + max|z|·|N×A|)/|A| + r·max(√m⁺ − 1, 1 − √(m⁻ − t²/A·A)),
//
// where |w|² lies in [m⁻, m⁺] = [min(|U|², |V|²) − |U·V|, max(|U|², |V|²) + |U·V|]
// and |w·A| ≤ t = |U·A| + |V·A|. Every reading is exact; each square root
// and the division by |A| round to the side that widens the bound. It is
// zero for an exactly orthonormal frame on the axis. It reports false when
// m⁻ − t²/A·A is not positive or the bound is not finite.
func boreToolDeviation(prism prismrecord.Payload, axis coneaxis.Axis, radius float64) (float64, bool) {
	frame := prism.Frame
	a := ratVec(axis.Vec())
	normSq := axis.NormSq().Rat()
	lo, _ := axis.NormBracket()
	if !(lo > 0) {
		return 0, false
	}
	u, v := ratVec(frame.U()), ratVec(frame.V())
	crossSq := func(p [3]*big.Rat) *big.Rat {
		c := ratCross(p, a)
		return ratDot(c, c)
	}
	zMax := max(math.Abs(prism.Z0), math.Abs(prism.Z1))
	offset := proofarith.FloatRat(proofbound.RatSqrtUp(crossSq(ratVec(frame.Origin()))))
	tilt := proofarith.FloatRat(proofbound.RatSqrtUp(crossSq(ratVec(frame.N()))))
	shift := new(big.Rat).Add(offset, new(big.Rat).Mul(proofarith.FloatRat(zMax), tilt))
	shift.Quo(shift, proofarith.FloatRat(lo))
	uu, vv, uv := ratDot(u, u), ratDot(v, v), new(big.Rat).Abs(ratDot(u, v))
	mHi := new(big.Rat).Add(proofbound.RatMax(uu, vv), uv)
	mLo := new(big.Rat).Sub(proofbound.RatMin(uu, vv), uv)
	t := new(big.Rat).Add(new(big.Rat).Abs(ratDot(u, a)), new(big.Rat).Abs(ratDot(v, a)))
	inner := new(big.Rat).Sub(mLo, new(big.Rat).Quo(new(big.Rat).Mul(t, t), normSq))
	if inner.Sign() <= 0 {
		return 0, false
	}
	one := big.NewRat(1, 1)
	outward := new(big.Rat).Sub(proofarith.FloatRat(proofbound.RatSqrtUp(mHi)), one)
	inward := new(big.Rat).Sub(one, proofarith.FloatRat(proofbound.RatSqrtDown(inner)))
	radial := proofbound.RatMax(proofbound.RatMax(outward, inward), new(big.Rat))
	radial.Mul(radial, proofarith.FloatRat(radius))
	deviation := proofbound.RatFloatUp(new(big.Rat).Add(shift, radial))
	return deviation, !proofbound.IsNonFinite(deviation) && deviation >= 0
}

func ratVec(v r3.Vec) [3]*big.Rat {
	return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

func ratDot(a, b [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for i := range a {
		out.Add(out, new(big.Rat).Mul(a[i], b[i]))
	}
	return out
}

func ratCross(a, b [3]*big.Rat) [3]*big.Rat {
	term := func(i, j, k, l int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(a[i], b[j]), new(big.Rat).Mul(a[k], b[l]))
	}
	return [3]*big.Rat{term(1, 2, 2, 1), term(2, 0, 0, 2), term(0, 1, 1, 0)}
}
