package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// A cylinder joined to a matching axial Loft keeps the original operands.
// Subsequent placements of that same Loft rebuild the common cylinder ring.
type loftCylinderJoinRecord struct {
	Blank     *topo.Body
	Placement *loftCylinderPlacement
	// Teeth are the exact Union operands. ProofTeeth are private, axially
	// normalized witnesses whose mesh proofs include AxialBridge.
	Teeth, ProofTeeth []*topo.Body
	AxialBridge       []float64
	CapOutlines       [2][]int
	CapGroups         [2]int
	MinSideRadius     float64
	Radius            float64
	// These upward bounds enclose the replaced root seam around Radius.
	// Retained tooth exterior walls still read their Loft source bounds.
	// The symmetric collar covers source-root exceptions near either cap.
	InnerInset, OuterExcess, RootCollar float64
	Z0, Z1                              float64
}

// The structural join has at most 32 separately certified Loft teeth. Its
// final crossing audit and the matched centered bore may scan twice the
// generic mesh ceiling; exact pair classifications remain unchanged.
const loftCylinderAuditMaxScans = 16_000_000

type loftCylinderTooth struct {
	body     *Body
	original *Body
	bridge   float64
	// upperSeparationBridge widens only the moving-plane separation proof
	// when a later mirrored assembly translates this tooth's upper source.
	upperSeparationBridge float64
	loft                  loftPayload
	rootStart             int
	bottom                [2]float64
	top                   [2]float64
	bottomIV              [2]proofbound.RatInterval
	topIV                 [2]proofbound.RatInterval
	mesh                  *Mesh
	vertex                [][]int
	rootGap               float64
	inset                 float64
	reach                 float64
	shift                 float64
	angular               float64
	reachOK               bool
	collar                float64
	wedge                 float64
}

func tryLoftCylinderUnion(ctx context.Context, op meshbool.OperationKind,
	d *Document, ref producerID, a, b *Body) (*Body, bool, error) {
	if op != meshbool.OpUnion {
		return nil, false, nil
	}
	var blank *Body
	var teeth []*Body
	if _, ok := a.n.Payload.(prismPayload); ok {
		blank, teeth = a, []*Body{b}
	} else if _, ok := b.n.Payload.(prismPayload); ok {
		blank, teeth = b, []*Body{a}
	} else if fp, ok := a.n.Payload.(facetedPayload); ok && fp.LoftCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftCylinderJoin.Blank)
		teeth = append(append([]*Body(nil), bodyHandles(fp.LoftCylinderJoin.Teeth)...), b)
	} else if fp, ok := b.n.Payload.(facetedPayload); ok && fp.LoftCylinderJoin != nil && fp.Xform == r3.Identity() {
		blank = bodyHandle(fp.LoftCylinderJoin.Blank)
		teeth = append(append([]*Body(nil), bodyHandles(fp.LoftCylinderJoin.Teeth)...), a)
	} else {
		return tryPlacedLoftCylinderUnion(ctx, d, ref, a, b)
	}
	body, ok, err := tryLoftCylinderUnionTeeth(ctx, d, ref, blank, teeth)
	if err != nil || ok {
		return body, ok, err
	}
	return tryPlacedLoftCylinderUnion(ctx, d, ref, a, b)
}

// tryLoftCylinderUnionTeeth certifies one complete circular Join from its
// original Loft operands. Multi-body Union uses this once for the full pattern.
func tryLoftCylinderUnionTeeth(ctx context.Context, d *Document, ref producerID,
	blank *Body, teeth []*Body) (*Body, bool, error) {
	return tryLoftCylinderUnionTeethWithUpperBridge(ctx, d, ref, blank, teeth, nil)
}

func tryLoftCylinderUnionTeethWithUpperBridge(ctx context.Context, d *Document, ref producerID,
	blank *Body, teeth []*Body, upperBridges []float64) (*Body, bool, error) {
	if upperBridges != nil && len(upperBridges) != len(teeth) {
		return nil, false, nil
	}
	pp, ok := blank.n.Payload.(prismPayload)
	if !ok || pp.SurfaceResult || pp.Xform != r3.Identity() ||
		pp.Z0Delta != 0 || pp.Z1Delta != 0 || pp.SectionDelta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.Frame.Origin() != (r3.Vec{}) || pp.Frame.U() != (r3.Vec{X: 1}) ||
		pp.Frame.V() != (r3.Vec{Y: 1}) || pp.Z1 <= pp.Z0 {
		return nil, false, nil
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || circle.Center != (planePoint{}) || !circle.CCW {
		return nil, false, nil
	}
	radius := circle.Radius.Base()
	if radius <= 0 || proofbound.IsNonFinite(radius) {
		return nil, false, nil
	}
	parts := make([]loftCylinderTooth, len(teeth))
	var source *loftrecord.SourceIdentity
	for i, tooth := range teeth {
		lp, ok := tooth.n.Payload.(loftPayload)
		if !ok || !lp.AuthenticatedReconstruction || lp.SurfaceResult ||
			lp.SourceIdentity == nil || len(lp.Profile0.Holes) != 0 ||
			len(lp.Profile1.Holes) != 0 || len(lp.Profile0.Outer.Segments) != 6 ||
			len(lp.Profile1.Outer.Segments) != 6 || len(lp.Segment) != 1 ||
			len(lp.Verts)%2 != 0 || len(lp.Verts) < 16 {
			return nil, false, nil
		}
		if source == nil {
			source = lp.SourceIdentity
		} else if source != lp.SourceIdentity {
			return nil, false, nil
		}
		if !loftCylinderSectionKinds(lp) || lp.Frame0.U() != (r3.Vec{X: 1}) ||
			lp.Frame0.V() != (r3.Vec{Y: 1}) || lp.Frame1.U() != (r3.Vec{X: 1}) ||
			lp.Frame1.V() != (r3.Vec{Y: 1}) ||
			lp.Frame0.Origin() != (r3.Vec{Z: pp.Z0}) ||
			lp.Frame1.Origin() != (r3.Vec{Z: pp.Z1}) {
			return nil, false, nil
		}
		proofTooth, bridge, admitted, err := loftCylinderProofTooth(ctx, d, tooth, pp.Z0, pp.Z1)
		if err != nil {
			return nil, true, err
		}
		if !admitted {
			return nil, false, nil
		}
		tooth = proofTooth
		lp, ok = tooth.n.Payload.(loftPayload)
		if !ok || !loftCylinderAxialPlacement(lp.Xform) {
			return nil, false, nil
		}
		n := len(lp.Verts) / 2
		if len(lp.Segment[0]) != n || lp.Walls != 2*n ||
			len(lp.Tris) != 4*n-4 {
			return nil, false, nil
		}
		root := -1
		for j, segment := range lp.Segment[0] {
			if segment == 5 && root < 0 {
				root = j
			}
			if (root < 0 && segment == 5) || (root >= 0 && segment != 5) {
				return nil, false, nil
			}
		}
		if root < 2 || root >= n-1 {
			return nil, false, nil
		}
		m, err := tooth.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
		if err != nil {
			return nil, false, err
		}
		if !m.BoundaryVerified() || !m.VolumeVerified() {
			return nil, false, nil
		}
		parts[i] = loftCylinderTooth{
			body: tooth, original: teeth[i], bridge: bridge,
			loft: lp, rootStart: root, mesh: m,
		}
		if upperBridges != nil {
			if upperBridges[i] < 0 || proofbound.IsNonFinite(upperBridges[i]) {
				return nil, false, nil
			}
			parts[i].upperSeparationBridge = upperBridges[i]
		}
		for cap := range 2 {
			offset := cap * n
			p0, p1 := lp.Verts[offset], lp.Verts[offset+root]
			start, end := math.Atan2(p0.Y, p0.X), math.Atan2(p1.Y, p1.X)
			startIV := proofbound.Atan2Interval(proofarith.FloatRat(p0.Y), proofarith.FloatRat(p0.X), false)
			endIV := proofbound.Atan2Interval(proofarith.FloatRat(p1.Y), proofarith.FloatRat(p1.X), false)
			for end <= start {
				end += 2 * math.Pi
				endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
			}
			if end-start <= 0 || end-start >= math.Pi/2 {
				return nil, false, nil
			}
			if cap == 0 {
				parts[i].bottom = [2]float64{start, end}
				parts[i].bottomIV = [2]proofbound.RatInterval{startIV, endIV}
			} else {
				for start-parts[i].bottom[0] > math.Pi {
					start -= 2 * math.Pi
					end -= 2 * math.Pi
					startIV = proofbound.IntervalSub(startIV, proofbound.TwoPiInterval())
					endIV = proofbound.IntervalSub(endIV, proofbound.TwoPiInterval())
				}
				for start-parts[i].bottom[0] < -math.Pi {
					start += 2 * math.Pi
					end += 2 * math.Pi
					startIV = proofbound.IntervalAdd(startIV, proofbound.TwoPiInterval())
					endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
				}
				parts[i].top = [2]float64{start, end}
				parts[i].topIV = [2]proofbound.RatInterval{startIV, endIV}
			}
		}
		gap, ok := loftcylinder.RootRadiusGap(lp.Xform, [2]momentinput.Profile{lp.Profile0, lp.Profile1}, radius)
		if !ok {
			return nil, false, nil
		}
		height := proofbound.RatFloatUp(new(big.Rat).Sub(
			proofarith.FloatRat(pp.Z1), proofarith.FloatRat(pp.Z0)))
		collar, ok := loftCylinderRootCollar(lp, radius, height)
		if !ok {
			return nil, false, nil
		}
		connectorCollar, ok := loftcylinder.ConnectorGate(lp.Verts, lp.Tris, lp.Segment, root, radius, height, m.Bound().Base())
		if !ok {
			return nil, false, nil
		}
		collar = max(collar, connectorCollar)
		reach, shift, angular, ok := loftcylinder.ConnectorReach(lp.Verts, lp.Tris, lp.Segment,
			[2]profileRecord{lp.Profile0, lp.Profile1}, lp.Xform.Basis().EX, lp.Proof.FacetDeparture, root, radius, collar)
		if ok {
			circle, err := loftCylinderSourceCircleOf(lp, radius)
			ok = err == nil && circle.OrderedOverSafeInterval()
		}
		reachOK := ok
		inset, ok := loftcylinder.RadialGate(lp.Verts, lp.Tris, lp.Segment, root, radius, m.Bound().Base(), gap)
		if !ok {
			return nil, false, nil
		}
		wedge, ok := loftcylinder.WedgeGate(lp.Verts, lp.Tris, root, pp.Z0, pp.Z1)
		if !ok {
			return nil, false, nil
		}
		parts[i].rootGap, parts[i].inset = gap, inset
		parts[i].reach, parts[i].shift, parts[i].angular = reach, shift, angular
		parts[i].reachOK = reachOK
		parts[i].collar, parts[i].wedge = collar, wedge
	}
	if len(parts) == 0 || len(parts) > 32 {
		return nil, false, nil
	}
	sort.Slice(parts, func(i, j int) bool { return parts[i].bottom[0] < parts[j].bottom[0] })
	if !loftCylinderSeparate(parts) {
		return nil, false, nil
	}
	body, err := buildLoftCylinderUnion(ctx, d, ref, blank, pp, radius, parts)
	return body, true, err
}

func loftCylinderSectionKinds(lp loftPayload) bool {
	for _, profile := range []profileRecord{lp.Profile0, lp.Profile1} {
		segments := profile.Outer.Segments
		start, ok := segments[0].(lineSeg)
		if !ok {
			return false
		}
		if _, ok := segments[1].(fitSplineSeg); !ok {
			return false
		}
		if _, ok := segments[2].(arcSeg); !ok {
			return false
		}
		if _, ok := segments[3].(fitSplineSeg); !ok {
			return false
		}
		end, ok := segments[4].(lineSeg)
		if !ok {
			return false
		}
		arc, ok := segments[5].(arcSeg)
		if !ok || arc.Center != (planePoint{}) || arc.TStart != 1 || arc.TEnd != 0 {
			return false
		}
		if start.TStart != 0 || start.TEnd != 1 || end.TStart != 0 || end.TEnd != 1 ||
			start.Start != arc.Start || end.End != arc.End {
			return false
		}
	}
	return true
}

func loftCylinderAxialPlacement(xform r3.Transform) bool {
	b := xform.Basis()
	return xform.Translation() == (r3.Vec{}) && b.EZ == (r3.Vec{Z: 1}) &&
		b.EX.Z == 0 && b.EY.Z == 0 && b.EX.X == b.EY.Y &&
		b.EX.Y == -b.EY.X
}

// A matched pair of centered circular arcs has a constant radius at each
// cap and a linearly interpolated angular difference along its parameter.
// At every arc parameter, squared radius along a Loft ruling is convex in
// height. Both interior endpoint inequalities certify the interval between
// the returned cap collars. A positive cap radius roundoff is admitted only
// when this proof places the root wall inside the blank between them.
func loftCylinderRootCollar(lp loftPayload, radius, height float64) (float64, bool) {
	var q [2]*big.Rat
	var angles [2][2]proofbound.RatInterval
	basis := lp.Xform.Basis()
	c, s := proofarith.FloatRat(basis.EX.X), proofarith.FloatRat(basis.EX.Y)
	scale2 := new(big.Rat).Add(new(big.Rat).Mul(c, c), new(big.Rat).Mul(s, s))
	for cap, profile := range []profileRecord{lp.Profile0, lp.Profile1} {
		arc, ok := profile.Outer.Segments[5].(arcSeg)
		if !ok {
			return 0, false
		}
		x, y := proofarith.FloatRat(arc.Start.U), proofarith.FloatRat(arc.Start.V)
		q[cap] = new(big.Rat).Mul(scale2,
			new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
		for end, p := range []planePoint{arc.Start, arc.End} {
			angles[cap][end] = proofbound.Atan2Interval(
				proofarith.FloatRat(p.V), proofarith.FloatRat(p.U), false)
		}
		if angles[cap][1].Hi.Cmp(angles[cap][0].Lo) <= 0 {
			angles[cap][1] = proofbound.IntervalAdd(angles[cap][1], proofbound.TwoPiInterval())
		}
	}
	r2 := new(big.Rat).Mul(proofarith.FloatRat(radius), proofarith.FloatRat(radius))
	if q[0].Cmp(r2) <= 0 && q[1].Cmp(r2) <= 0 {
		return 0, true
	}
	minTwist := new(big.Rat)
	twistSign := 0
	for end := range 2 {
		delta := proofbound.IntervalSub(angles[1][end], angles[0][end])
		var lower *big.Rat
		sign := 0
		switch {
		case delta.Lo.Sign() > 0 && delta.Hi.Cmp(proofarith.FloatRat(0.4)) < 0:
			lower = delta.Lo
			sign = 1
		case delta.Hi.Sign() < 0 && delta.Lo.Cmp(proofarith.FloatRat(-0.4)) > 0:
			lower = new(big.Rat).Neg(delta.Hi)
			sign = -1
		default:
			return 0, false
		}
		if twistSign != 0 && sign != twistSign {
			return 0, false
		}
		twistSign = sign
		if end == 0 || lower.Cmp(minTwist) < 0 {
			minTwist = lower
		}
	}
	_, cosine, ok := proofbound.RadSinCosInterval(minTwist)
	if !ok {
		return 0, false
	}
	oneMinusCos := new(big.Rat).Sub(big.NewRat(1, 1), cosine.Hi)
	if oneMinusCos.Sign() <= 0 {
		return 0, false
	}
	rProductLo := proofarith.FloatRat(proofbound.RatSqrtDown(
		new(big.Rat).Mul(q[0], q[1])))
	for shift := 52; shift >= 8; shift-- {
		epsilon := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), uint(shift)))
		oneMinus := new(big.Rat).Sub(big.NewRat(1, 1), epsilon)
		inward := new(big.Rat).Mul(big.NewRat(2, 1), epsilon)
		inward.Mul(inward, oneMinus)
		inward.Mul(inward, rProductLo)
		inward.Mul(inward, oneMinusCos)
		bottom := new(big.Rat).Add(new(big.Rat).Mul(oneMinus, q[0]),
			new(big.Rat).Mul(epsilon, q[1]))
		top := new(big.Rat).Add(new(big.Rat).Mul(epsilon, q[0]),
			new(big.Rat).Mul(oneMinus, q[1]))
		if bottom.Sub(bottom, inward).Cmp(r2) < 0 && top.Sub(top, inward).Cmp(r2) < 0 {
			collar := proofbound.ProductUpper(proofbound.RatFloatUp(epsilon), height)
			return collar, !proofbound.IsNonFinite(collar)
		}
	}
	return 0, false
}

func loftCylinderSeparate(parts []loftCylinderTooth) bool {
	// Try a fixed through-axis plane first. A twisted pair may need a plane
	// whose normal changes with height; that case checks whole triangles.
	movingWork := 8_000_000
	for i := range parts {
		for j := i + 1; j < len(parts); j++ {
			center := func(part loftCylinderTooth) (float64, float64) {
				angle := (part.bottom[0] + part.bottom[1] +
					part.top[0] + part.top[1]) / 4
				return math.Cos(angle), math.Sin(angle)
			}
			ax, ay := center(parts[i])
			bx, by := center(parts[j])
			nx, ny := bx-ax, by-ay
			if !proofbound.FiniteVec(r3.Vec{X: nx, Y: ny}) {
				return false
			}
			rx, ry := proofarith.FloatRat(nx), proofarith.FloatRat(ny)
			length2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx),
				new(big.Rat).Mul(ry, ry))
			length := proofbound.RatSqrtUp(length2)
			if length <= 0 || proofbound.IsNonFinite(length) {
				return false
			}
			fixed := true
			for k, part := range []loftCylinderTooth{parts[i], parts[j]} {
				margin := proofbound.ProductUpper(length, part.mesh.Bound().Base())
				marginRat := proofarith.FloatRat(margin)
				for _, p := range part.loft.Verts {
					dot := new(big.Rat).Add(new(big.Rat).Mul(rx, proofarith.FloatRat(p.X)),
						new(big.Rat).Mul(ry, proofarith.FloatRat(p.Y)))
					if k == 0 && dot.Cmp(new(big.Rat).Neg(marginRat)) >= 0 ||
						k == 1 && dot.Cmp(marginRat) <= 0 {
						fixed = false
						break
					}
				}
				if !fixed {
					break
				}
			}
			if !fixed && !loftCylinderMovingSeparate(parts[i], parts[j], &movingWork) {
				return false
			}
		}
	}
	for i := range parts {
		j := (i + 1) % len(parts)
		for cap := range 2 {
			a, b := parts[i].bottom, parts[j].bottom
			if cap == 1 {
				a, b = parts[i].top, parts[j].top
			}
			for b[0] <= a[0] {
				b[0] += 2 * math.Pi
				b[1] += 2 * math.Pi
			}
			if a[1]+0.02 >= b[0] || b[0]-a[1] >= 2*math.Pi {
				return false
			}
		}
	}
	return true
}

func buildLoftCylinderUnion(ctx context.Context, d *Document, ref producerID,
	blank *Body, pp prismPayload, radius float64, parts []loftCylinderTooth) (*Body, error) {
	mesh := &joinedMesh{}
	blankFaces := blank.Faces()
	if len(blankFaces) != 3 {
		return nil, fmt.Errorf("%w: a circular blank has changed face topology", ErrUnsupported)
	}
	blankGroups := [3]int{-1, -1, -1}
	for _, f := range blankFaces {
		role := f.Origins()[0].Role
		group := -1
		switch role {
		case prismrecord.OuterFirstSideRole:
			group = 0
		case provenance.RoleCapStart:
			group = 1
		case provenance.RoleCapEnd:
			group = 2
		}
		if group < 0 || blankGroups[group] >= 0 {
			return nil, fmt.Errorf("%w: a circular blank has changed face roles", ErrUnsupported)
		}
		blankGroups[group] = len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetGroup{Origins: f.originRefs(), Planar: group != 0})
	}
	for _, group := range blankGroups {
		if group < 0 {
			return nil, fmt.Errorf("%w: a circular blank lost a source face", ErrUnsupported)
		}
	}
	toothBound, toothVolume, toothArea := 0.0, 0.0, 0.0
	for i := range parts {
		part := &parts[i]
		lp, n := part.loft, len(part.loft.Verts)/2
		part.vertex = make([][]int, 2)
		for cap := range 2 {
			part.vertex[cap] = make([]int, part.rootStart+1)
			for j := range part.vertex[cap] {
				part.vertex[cap][j] = len(mesh.Verts)
				mesh.Verts = append(mesh.Verts, lp.Verts[cap*n+j])
			}
		}
		faces := part.body.Faces()
		if len(faces) != lp.Walls+2 {
			return nil, fmt.Errorf("%w: a loft has changed face topology", ErrUnsupported)
		}
		for j := range part.rootStart {
			for side := range 2 {
				k := 2*j + side
				f := faces[2+k]
				if len(f.Origins()) == 0 || f.Origins()[0].Role != fmt.Sprintf("side(0,%d,%d)", j, side) {
					return nil, fmt.Errorf("%w: a loft wall lost its source role", ErrUnsupported)
				}
				tri := lp.Tris[k]
				var mapped [3]int
				for corner, old := range tri {
					capIndex, station := old/n, old%n
					if capIndex > 1 || station > part.rootStart {
						return nil, fmt.Errorf("%w: a loft wall crosses its root arc", ErrUnsupported)
					}
					mapped[corner] = part.vertex[capIndex][station]
				}
				group := len(mesh.Groups)
				mesh.Groups = append(mesh.Groups, facetGroup{Origins: f.originRefs(), Planar: true})
				mesh.Add(mapped, group)
			}
		}
		toothBound = max(toothBound, part.mesh.Bound().Base())
		toothVolume = proofbound.AbsSumUpper(toothVolume, part.mesh.n.VolSymDiff)
		area, err := part.body.Area()
		if err != nil {
			return nil, err
		}
		toothArea = proofbound.AbsSumUpper(toothArea, area.Value.Base(), area.Bound.Base())
	}
	var outlines [2][]int
	maximumStep, maximumTwist, ringRound := 0.0, 0.0, 0.0
	for i := range parts {
		part := &parts[i]
		next := &parts[(i+1)%len(parts)]
		for cap := range 2 {
			outlines[cap] = append(outlines[cap], part.vertex[cap]...)
		}
		var gap [2][2]float64
		var gapIV [2][2]proofbound.RatInterval
		for cap := range 2 {
			a, b := part.bottom, next.bottom
			ai, bi := part.bottomIV, next.bottomIV
			if cap == 1 {
				a, b = part.top, next.top
				ai, bi = part.topIV, next.topIV
			}
			for b[0] <= a[0] {
				b[0] += 2 * math.Pi
				bi[0] = proofbound.IntervalAdd(bi[0], proofbound.TwoPiInterval())
			}
			gap[cap] = [2]float64{a[1], b[0]}
			gapIV[cap] = [2]proofbound.RatInterval{ai[1], bi[0]}
		}
		width := max(gap[0][1]-gap[0][0], gap[1][1]-gap[1][0])
		steps := int(math.Ceil(width / 0.02))
		if steps < 1 || steps > 512 {
			return nil, fmt.Errorf("%w: circular gap exceeds its chord budget", ErrUnsupported)
		}
		maximumStep = max(maximumStep, width/float64(steps))
		var rings [2][]int
		for cap := range 2 {
			rings[cap] = make([]int, steps+1)
			rings[cap][0] = part.vertex[cap][part.rootStart]
			rings[cap][steps] = next.vertex[cap][0]
			for step := 1; step < steps; step++ {
				t := float64(step) / float64(steps)
				angle := gap[cap][0] + t*(gap[cap][1]-gap[cap][0])
				fraction := big.NewRat(int64(step), int64(steps))
				trueAngle := proofbound.IntervalAdd(
					proofbound.IntervalScale(gapIV[cap][0], new(big.Rat).Sub(big.NewRat(1, 1), fraction)),
					proofbound.IntervalScale(gapIV[cap][1], fraction))
				angleError := proofbound.IntervalFloatError(trueAngle, angle)
				z := pp.Z0
				if cap == 1 {
					z = pp.Z1
				}
				p := r3.Vec{X: radius * math.Cos(angle), Y: radius * math.Sin(angle), Z: z}
				if !proofbound.FiniteVec(p) {
					return nil, fmt.Errorf("%w: circular ring is non-finite", ErrUnsupported)
				}
				sine, cosine, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
				if !ok {
					return nil, fmt.Errorf("%w: circular ring has no trigonometric enclosure", ErrUnsupported)
				}
				r := proofarith.FloatRat(radius)
				pointRound := proofbound.Radius3D(max(
					proofbound.IntervalFloatError(proofbound.IntervalScale(cosine, r), p.X),
					proofbound.IntervalFloatError(proofbound.IntervalScale(sine, r), p.Y)))
				ringRound = max(ringRound, proofbound.AbsSumUpper(pointRound,
					proofbound.ProductUpper(radius, angleError)))
				rings[cap][step] = len(mesh.Verts)
				mesh.Verts = append(mesh.Verts, p)
				outlines[cap] = append(outlines[cap], rings[cap][step])
			}
		}
		maximumTwist = max(maximumTwist,
			math.Abs(gap[1][0]-gap[0][0]), math.Abs(gap[1][1]-gap[0][1]))
		for step := range steps {
			b0, b1 := rings[0][step], rings[0][step+1]
			t0, t1 := rings[1][step], rings[1][step+1]
			if !loftcylinder.RingOrder(mesh.Verts[b0], mesh.Verts[b1],
				mesh.Verts[t0], mesh.Verts[t1]) {
				return nil, fmt.Errorf("%w: circular ring loses angular order through height", ErrUnsupported)
			}
			mesh.Add([3]int{b0, b1, t1}, blankGroups[0])
			mesh.Add([3]int{b0, t1, t0}, blankGroups[0])
		}
	}
	for cap, outline := range outlines {
		points := make([]triangulation.Point2, len(outline))
		loop := make([]int, len(outline))
		for i, index := range outline {
			p := mesh.Verts[index]
			points[i] = triangulation.Point2{U: p.X, V: p.Y}
			loop[i] = i
		}
		triangles, err := triangulation.Triangulate(ctx, points, [][]int{loop})
		if err != nil {
			return nil, triangulation.WrapLoftError(err)
		}
		for _, tri := range triangles {
			mapped := [3]int{outline[tri[0]], outline[tri[1]], outline[tri[2]]}
			if cap == 0 {
				mapped[1], mapped[2] = mapped[2], mapped[1]
			}
			mesh.Add(mapped, blankGroups[cap+1])
		}
	}
	if maximumTwist > 0.4 || maximumStep > 0.025 {
		return nil, fmt.Errorf("%w: circular join exceeds its angular proof limit", ErrUnsupported)
	}
	// Every omitted root wall and every replacement cylinder facet lies in an
	// annulus. The held root facets' exact projected minima and the new
	// cylinder facets' chord sagitta both contribute to its inner edge.
	width := proofbound.AbsSumUpper(maximumTwist, maximumStep)
	seamInset := proofbound.ProductUpper(radius,
		proofbound.ProductUpper(width, width)/8)
	innerInset, outerExcess, rootCollar, connectorReach := seamInset, ringRound, 0.0, 0.0
	crossingShift, rootAngular := 0.0, 0.0
	allReach := true
	// RatSqrtDown, RatSqrtUp and RatFloatUp are monotone, so the exact extreme
	// over the ring is converted once instead of once per triangle.
	var ringMin2, ringMax2 *big.Rat
	for i, tri := range mesh.Tris {
		if mesh.Src[i] != blankGroups[0] {
			continue
		}
		minimum2 := loftcylinder.TriangleMinRadius2(mesh.Verts, tri)
		if ringMin2 == nil || minimum2.Cmp(ringMin2) < 0 {
			ringMin2 = minimum2
		}
		for _, index := range tri {
			p := mesh.Verts[index]
			x, y := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)
			q := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
			if ringMax2 == nil || q.Cmp(ringMax2) > 0 {
				ringMax2 = q
			}
		}
	}
	if ringMin2 != nil {
		minimum := proofbound.RatSqrtDown(ringMin2)
		if minimum <= 0 {
			return nil, fmt.Errorf("%w: circular ring reaches the axis", ErrUnsupported)
		}
		innerInset = max(innerInset, proofbound.RatFloatUp(new(big.Rat).Sub(
			proofarith.FloatRat(radius), proofarith.FloatRat(minimum))))
		maximum := proofbound.RatSqrtUp(ringMax2)
		outerExcess = max(outerExcess, proofbound.RatFloatUp(new(big.Rat).Sub(
			proofarith.FloatRat(maximum), proofarith.FloatRat(radius))))
	}
	for _, part := range parts {
		allReach = allReach && part.reachOK
		innerInset = max(innerInset, part.inset)
		innerInset = max(innerInset,
			proofbound.AbsSumUpper(part.wedge, part.mesh.Bound().Base()))
		connectorReach = max(connectorReach,
			proofbound.AbsSumUpper(part.reach, part.mesh.Bound().Base()))
		crossingShift = max(crossingShift, part.shift)
		rootAngular = max(rootAngular, part.angular)
		outerExcess = max(outerExcess,
			proofbound.AbsSumUpper(part.rootGap, part.mesh.Bound().Base()))
		rootCollar = max(rootCollar, part.collar)
	}
	blankMesh, err := blank.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		return nil, err
	}
	// In the angular disagreement between source and held tooth sectors, a
	// held ring point projects to the cylinder and then follows it to the
	// source crossing. The latter leg starts at the held root ray, not the
	// held crossing ray. Source cylinder points under a held tooth use the
	// separate source-to-held crossing shift. Hidden connector points use
	// their same-height chain directly.
	seamReach := proofbound.ProductUpper(2, innerInset)
	if allReach {
		seamReach = max(proofbound.AbsSumUpper(innerInset, rootAngular),
			connectorReach, crossingShift)
	}
	collarSweep, ok := loftCylinderCollarSweep(parts, pp.Z0, pp.Z1, rootCollar)
	if !ok {
		return nil, fmt.Errorf("%w: circular join lacks a cap-collar sweep bound", ErrUnsupported)
	}
	bound := proofbound.AbsSumUpper(toothBound, seamReach, outerExcess, rootCollar, collarSweep)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return nil, fmt.Errorf("%w: circular join exceeds the 0.1 mm boundary proof: bound %g blank %g tooth %g inset %g round %g twist %g step %g", ErrUnsupported, bound, blankMesh.Bound().Base(), toothBound, seamInset, ringRound, maximumTwist, maximumStep)
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris,
		loftmesh.MaxLoftAuditTriangles, loftCylinderAuditMaxScans); err != nil {
		return nil, err
	}
	for cap, outline := range outlines {
		z := pp.Z0
		if cap == 1 {
			z = pp.Z1
		}
		if !loftCylinderAuditCap(ctx, mesh, outline, blankGroups[cap+1], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular join cap does not tile its recorded outline", ErrUnsupported)
		}
		if !loftCylinderRootChordsInsideCap(ctx, mesh, outline, parts, cap) {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			return nil, fmt.Errorf("%w: circular join root chord leaves its cap", ErrUnsupported)
		}
	}
	var sideMin2 *big.Rat
	for i, tri := range mesh.Tris {
		if mesh.Src[i] == blankGroups[1] || mesh.Src[i] == blankGroups[2] {
			continue
		}
		if q := loftcylinder.TriangleMinRadius2(mesh.Verts, tri); sideMin2 == nil || q.Cmp(sideMin2) < 0 {
			sideMin2 = q
		}
	}
	minSideRadius := math.Inf(1)
	if sideMin2 != nil {
		minSideRadius = proofbound.RatSqrtDown(sideMin2)
	}
	if minSideRadius <= 0 || proofbound.IsNonFinite(minSideRadius) {
		return nil, fmt.Errorf("%w: circular join has no side clearance certificate", ErrUnsupported)
	}
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return nil, err
	}
	blankArea, err := blank.Area()
	if err != nil {
		return nil, err
	}
	// Outside this full radial band, the blank is either wholly occupied or
	// wholly absent, and the retained Loft facets have the same source roles.
	// Charge the entire band instead of assuming a facet correspondence at
	// the root seam. The source operands' own occupied-volume errors remain.
	if innerInset >= radius {
		return nil, fmt.Errorf("%w: circular join reaches the axis", ErrUnsupported)
	}
	bandWidth := proofbound.AbsSumUpper(innerInset, outerExcess)
	bandArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(2, radius), bandWidth),
		proofbound.ProductUpper(outerExcess, outerExcess))
	annulusArea := proofbound.ProductUpper(proofbound.RatFloatUp(proofbound.PiUpper), bandArea)
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(pp.Z1), proofarith.FloatRat(pp.Z0)))
	annulusVolume := proofbound.ProductUpper(annulusArea, height)
	volumeGap := proofbound.AbsSumUpper(blankMesh.n.VolSymDiff, toothVolume, annulusVolume)
	areaSlack := proofbound.AbsSumUpper(area, blankArea.Value.Base(), blankArea.Bound.Base(), toothArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return nil, fmt.Errorf("%w: circular join has no finite volume proof", ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	toothBodies := make([]*Body, len(parts))
	proofBodies := make([]*Body, len(parts))
	axialBridge := make([]float64, len(parts))
	for i := range parts {
		toothBodies[i] = parts[i].original
		proofBodies[i] = parts[i].body
		axialBridge[i] = parts[i].bridge
	}
	fp := facetedPayload{Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src,
		Groups: mesh.Groups, VertexBound: vertexBound, ContactAudited: true,
		MeshBound: bound, VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: facetedSourceDiameterUpper(mesh.Verts, bound),
		Xform: r3.Identity(), LoftCylinderJoin: &loftCylinderJoinRecord{
			Blank: bodyNode(blank), Teeth: bodyNodes(toothBodies), ProofTeeth: bodyNodes(proofBodies),
			AxialBridge: axialBridge, CapOutlines: outlines,
			CapGroups:     [2]int{blankGroups[1], blankGroups[2]},
			MinSideRadius: minSideRadius, Radius: radius, Z0: pp.Z0, Z1: pp.Z1,
			InnerInset: innerInset, OuterExcess: outerExcess, RootCollar: rootCollar,
		}}
	if proofbound.IsNonFinite(fp.DPair) {
		return nil, fmt.Errorf("%w: circular join has no finite diameter proof", ErrUnsupported)
	}
	if err := loftCylinderTagCaps(&fp, blank, fp.LoftCylinderJoin.CapGroups,
		[2]float64{pp.Z0, pp.Z1}, ref); err != nil {
		return nil, err
	}
	loftCylinderTagBridgedTop(&fp, fp.LoftCylinderJoin.CapGroups[1], axialBridge)
	return buildFacetedBody(ctx, d, ref, fp)
}

// Source root and connector rulings move linearly between the cap sections.
// A cap-collar point is at most collar/height of its whole XY travel from
// the matching cap point. Source wall positions lie within the held wall's
// matched Loft bound, so the paired held vertices give a radius ceiling.
func loftCylinderCollarSweep(parts []loftCylinderTooth, z0, z1, collar float64) (float64, bool) {
	if collar == 0 {
		return 0, true
	}
	height := new(big.Rat).Sub(proofarith.FloatRat(z1), proofarith.FloatRat(z0))
	if height.Sign() <= 0 || collar < 0 || proofbound.IsNonFinite(collar) {
		return 0, false
	}
	maximum := 0.0
	for _, part := range parts {
		toothMaximum := 0.0
		for _, p := range part.loft.Verts {
			x, y := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)
			radius := proofbound.RatSqrtUp(new(big.Rat).Add(
				new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
			toothMaximum = max(toothMaximum, radius)
		}
		maximum = max(maximum, proofbound.AbsSumUpper(toothMaximum, part.loft.Proof.FacetDeparture))
	}
	ratio := proofbound.RatFloatUp(new(big.Rat).Quo(proofarith.FloatRat(collar), height))
	sweep := proofbound.ProductUpper(proofbound.ProductUpper(2, maximum), ratio)
	return sweep, !proofbound.IsNonFinite(sweep)
}

// The cap audit establishes a simple filled polygon. Each omitted held root
// chord stays in that polygon if one interior point is strictly inside and
// the chord has no interior contact with its outline. Matched source root
// arcs then use their authenticated Loft departure to reach the cap.
func loftCylinderRootChordsInsideCap(ctx context.Context, mesh *joinedMesh,
	outline []int, parts []loftCylinderTooth, capIndex int) bool {
	if capIndex < 0 || capIndex > 1 || len(outline) < 3 {
		return false
	}
	points := make([]loftcylinder.XY, len(outline))
	poly := make([]meshbool.Xp2, len(outline))
	for i, index := range outline {
		if index < 0 || index >= len(mesh.Verts) {
			return false
		}
		points[i] = loftcylinder.ExactXY(mesh.Verts[index])
		poly[i] = meshbool.NewXP2(points[i].X, points[i].Y)
	}
	same := func(a, b loftcylinder.XY) bool {
		return a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, part := range parts {
		lp := part.loft
		n := len(lp.Verts) / 2
		if n < 2 || part.rootStart < 0 || part.rootStart >= n-1 {
			return false
		}
		rootA := loftcylinder.ExactXY(lp.Verts[capIndex*n+part.rootStart])
		rootB := loftcylinder.ExactXY(lp.Verts[capIndex*n])
		for j := part.rootStart; j < n; j++ {
			if ctx.Err() != nil {
				return false
			}
			a := loftcylinder.ExactXY(lp.Verts[capIndex*n+j])
			b := loftcylinder.ExactXY(lp.Verts[capIndex*n+(j+1)%n])
			if same(a, b) {
				return false
			}
			mid := meshbool.NewXP2(
				new(big.Rat).Quo(new(big.Rat).Add(a.X, b.X), big.NewRat(2, 1)),
				new(big.Rat).Quo(new(big.Rat).Add(a.Y, b.Y), big.NewRat(2, 1)))
			inside, on, err := meshbool.PointInPoly2(budget, poly, mid)
			if err != nil || on || !inside {
				return false
			}
			for k, c := range points {
				d := points[(k+1)%len(points)]
				if !loftcylinder.EdgesMeet(a, b, c, d) {
					continue
				}
				// A non-collinear outline edge may touch only a shared root
				// endpoint. All other contact, including overlap, refuses.
				shared := (same(a, rootA) || same(a, rootB)) && (same(a, c) || same(a, d)) ||
					(same(b, rootA) || same(b, rootB)) && (same(b, c) || same(b, d))
				if !shared || loftcylinder.Orient(a, b, c).Sign() == 0 &&
					loftcylinder.Orient(a, b, d).Sign() == 0 {
					return false
				}
			}
		}
	}
	return true
}
