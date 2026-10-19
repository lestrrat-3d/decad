package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

// Join builds and proves the shell of one circular Join: the blank's side
// ring and cap disks around the retained exterior walls of separated teeth.
// Shell builds the shell; Payload proves it once the caller has the blank's
// own verified mesh.
type Join struct {
	// Blank is the exact circular blank. Originals are the exact Union
	// operands and Proofs their private proof teeth, both indexed by
	// Tooth.Operand.
	Blank     *topo.Body
	Originals []*topo.Body
	Proofs    []*topo.Body
	// Teeth are the admitted teeth in the order Separate leaves them.
	Teeth  []Tooth
	Radius float64
	Z0, Z1 float64

	mesh     facetedrecord.JoinedMesh
	groups   [3]int // side, bottom cap and top cap
	outlines [2][]int

	toothBound, toothVolume, toothArea float64
	step, twist, round, seamInset      float64

	innerInset, outerExcess, rootCollar    float64
	connectorReach, crossingShift, angular float64
	allReach                               bool
}

// Shell builds the joined mesh: the blank's three source groups, each
// tooth's exterior walls up to its root, and the replacement side ring and
// cap disks. It also gathers the seam bounds Payload charges.
func (j *Join) Shell(ctx context.Context) error {
	if err := j.sourceGroups(); err != nil {
		return err
	}
	if err := j.ring(ctx); err != nil {
		return err
	}
	return j.seam()
}

// sourceGroups adds the blank's side and cap groups and every tooth's
// retained exterior walls.
func (j *Join) sourceGroups() error {
	mesh := &j.mesh
	blankFaces := topobuild.Faces(j.Blank)
	if len(blankFaces) != 3 {
		return fmt.Errorf("%w: a circular blank has changed face topology", decaderr.ErrUnsupported)
	}
	j.groups = [3]int{-1, -1, -1}
	for _, f := range blankFaces {
		group := -1
		switch f.Origins[0].Role {
		case prismrecord.OuterFirstSideRole:
			group = 0
		case provenance.RoleCapStart:
			group = 1
		case provenance.RoleCapEnd:
			group = 2
		}
		if group < 0 || j.groups[group] >= 0 {
			return fmt.Errorf("%w: a circular blank has changed face roles", decaderr.ErrUnsupported)
		}
		j.groups[group] = len(mesh.Groups)
		mesh.Groups = append(mesh.Groups, facetedrecord.Group{
			Origins: append([]provenance.Ref(nil), f.Origins...), Planar: group != 0})
	}
	for _, group := range j.groups {
		if group < 0 {
			return fmt.Errorf("%w: a circular blank lost a source face", decaderr.ErrUnsupported)
		}
	}
	for i := range j.Teeth {
		part := &j.Teeth[i]
		lp, n := part.Loft, len(part.Loft.Verts)/2
		for cap := range 2 {
			part.Vertex[cap] = make([]int, part.RootStart+1)
			for k := range part.Vertex[cap] {
				part.Vertex[cap][k] = len(mesh.Verts)
				mesh.Verts = append(mesh.Verts, lp.Verts[cap*n+k])
			}
		}
		proof := j.Proofs[part.Operand]
		faces := topobuild.Faces(proof)
		if len(faces) != lp.Walls+2 {
			return fmt.Errorf("%w: a loft has changed face topology", decaderr.ErrUnsupported)
		}
		for station := range part.RootStart {
			for side := range 2 {
				k := 2*station + side
				f := faces[2+k]
				if len(f.Origins) == 0 || f.Origins[0].Role != fmt.Sprintf("side(0,%d,%d)", station, side) {
					return fmt.Errorf("%w: a loft wall lost its source role", decaderr.ErrUnsupported)
				}
				var mapped [3]int
				for corner, old := range lp.Tris[k] {
					capIndex, held := old/n, old%n
					if capIndex > 1 || held > part.RootStart {
						return fmt.Errorf("%w: a loft wall crosses its root arc", decaderr.ErrUnsupported)
					}
					mapped[corner] = part.Vertex[capIndex][held]
				}
				group := len(mesh.Groups)
				mesh.Groups = append(mesh.Groups, facetedrecord.Group{
					Origins: append([]provenance.Ref(nil), f.Origins...), Planar: true})
				mesh.Add(mapped, group)
			}
		}
		j.toothBound = max(j.toothBound, part.MeshBound)
		j.toothVolume = proofbound.AbsSumUpper(j.toothVolume, part.MeshVolSymDiff)
		j.toothArea = proofbound.AbsSumUpper(j.toothArea, proof.Area.Value.Base(), proof.Area.Bound.Base())
	}
	return nil
}

// ring fills each gap between neighbouring teeth with cylinder rulings and
// tiles both cap outlines.
func (j *Join) ring(ctx context.Context) error {
	mesh := &j.mesh
	parts := j.Teeth
	for i := range parts {
		part := &parts[i]
		next := &parts[(i+1)%len(parts)]
		for cap := range 2 {
			j.outlines[cap] = append(j.outlines[cap], part.Vertex[cap]...)
		}
		var gap [2][2]float64
		var gapIV [2][2]proofbound.RatInterval
		for cap := range 2 {
			a, b := part.Bottom, next.Bottom
			ai, bi := part.BottomIV, next.BottomIV
			if cap == 1 {
				a, b = part.Top, next.Top
				ai, bi = part.TopIV, next.TopIV
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
			return fmt.Errorf("%w: circular gap exceeds its chord budget", decaderr.ErrUnsupported)
		}
		j.step = max(j.step, width/float64(steps))
		var rings [2][]int
		for cap := range 2 {
			rings[cap] = make([]int, steps+1)
			rings[cap][0] = part.Vertex[cap][part.RootStart]
			rings[cap][steps] = next.Vertex[cap][0]
			for step := 1; step < steps; step++ {
				t := float64(step) / float64(steps)
				angle := gap[cap][0] + t*(gap[cap][1]-gap[cap][0])
				fraction := big.NewRat(int64(step), int64(steps))
				trueAngle := proofbound.IntervalAdd(
					proofbound.IntervalScale(gapIV[cap][0], new(big.Rat).Sub(big.NewRat(1, 1), fraction)),
					proofbound.IntervalScale(gapIV[cap][1], fraction))
				angleError := proofbound.IntervalFloatError(trueAngle, angle)
				z := j.Z0
				if cap == 1 {
					z = j.Z1
				}
				p := r3.Vec{X: j.Radius * math.Cos(angle), Y: j.Radius * math.Sin(angle), Z: z}
				if !proofbound.FiniteVec(p) {
					return fmt.Errorf("%w: circular ring is non-finite", decaderr.ErrUnsupported)
				}
				sine, cosine, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(angle))
				if !ok {
					return fmt.Errorf("%w: circular ring has no trigonometric enclosure", decaderr.ErrUnsupported)
				}
				r := proofarith.FloatRat(j.Radius)
				pointRound := proofbound.Radius3D(max(
					proofbound.IntervalFloatError(proofbound.IntervalScale(cosine, r), p.X),
					proofbound.IntervalFloatError(proofbound.IntervalScale(sine, r), p.Y)))
				j.round = max(j.round, proofbound.AbsSumUpper(pointRound,
					proofbound.ProductUpper(j.Radius, angleError)))
				rings[cap][step] = len(mesh.Verts)
				mesh.Verts = append(mesh.Verts, p)
				j.outlines[cap] = append(j.outlines[cap], rings[cap][step])
			}
		}
		j.twist = max(j.twist,
			math.Abs(gap[1][0]-gap[0][0]), math.Abs(gap[1][1]-gap[0][1]))
		for step := range steps {
			b0, b1 := rings[0][step], rings[0][step+1]
			t0, t1 := rings[1][step], rings[1][step+1]
			if !RingOrder(mesh.Verts[b0], mesh.Verts[b1],
				mesh.Verts[t0], mesh.Verts[t1]) {
				return fmt.Errorf("%w: circular ring loses angular order through height", decaderr.ErrUnsupported)
			}
			mesh.Add([3]int{b0, b1, t1}, j.groups[0])
			mesh.Add([3]int{b0, t1, t0}, j.groups[0])
		}
	}
	for cap, outline := range j.outlines {
		points := make([]triangulation.Point2, len(outline))
		loop := make([]int, len(outline))
		for i, index := range outline {
			p := mesh.Verts[index]
			points[i] = triangulation.Point2{U: p.X, V: p.Y}
			loop[i] = i
		}
		triangles, err := triangulation.Triangulate(ctx, points, [][]int{loop})
		if err != nil {
			return triangulation.WrapLoftError(err)
		}
		for _, tri := range triangles {
			mapped := [3]int{outline[tri[0]], outline[tri[1]], outline[tri[2]]}
			if cap == 0 {
				mapped[1], mapped[2] = mapped[2], mapped[1]
			}
			mesh.Add(mapped, j.groups[cap+1])
		}
	}
	if j.twist > 0.4 || j.step > 0.025 {
		return fmt.Errorf("%w: circular join exceeds its angular proof limit", decaderr.ErrUnsupported)
	}
	return nil
}

// seam bounds the radial band that holds every omitted root wall and every
// replacement cylinder facet, and gathers the teeth's seam gate results.
func (j *Join) seam() error {
	mesh := &j.mesh
	// The held root facets' exact projected minima and the new cylinder
	// facets' chord sagitta both contribute to the band's inner edge.
	width := proofbound.AbsSumUpper(j.twist, j.step)
	j.seamInset = proofbound.ProductUpper(j.Radius,
		proofbound.ProductUpper(width, width)/8)
	j.innerInset, j.outerExcess = j.seamInset, j.round
	j.allReach = true
	// RatSqrtDown, RatSqrtUp and RatFloatUp are monotone, so the exact extreme
	// over the ring is converted once instead of once per triangle.
	var ringMin2, ringMax2 *big.Rat
	for i, tri := range mesh.Tris {
		if mesh.Src[i] != j.groups[0] {
			continue
		}
		minimum2 := TriangleMinRadius2(mesh.Verts, tri)
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
			return fmt.Errorf("%w: circular ring reaches the axis", decaderr.ErrUnsupported)
		}
		j.innerInset = max(j.innerInset, proofbound.RatFloatUp(new(big.Rat).Sub(
			proofarith.FloatRat(j.Radius), proofarith.FloatRat(minimum))))
		maximum := proofbound.RatSqrtUp(ringMax2)
		j.outerExcess = max(j.outerExcess, proofbound.RatFloatUp(new(big.Rat).Sub(
			proofarith.FloatRat(maximum), proofarith.FloatRat(j.Radius))))
	}
	for _, part := range j.Teeth {
		j.allReach = j.allReach && part.ReachOK
		j.innerInset = max(j.innerInset, part.Inset)
		j.innerInset = max(j.innerInset,
			proofbound.AbsSumUpper(part.Wedge, part.MeshBound))
		j.connectorReach = max(j.connectorReach,
			proofbound.AbsSumUpper(part.Reach, part.MeshBound))
		j.crossingShift = max(j.crossingShift, part.Shift)
		j.angular = max(j.angular, part.Angular)
		j.outerExcess = max(j.outerExcess,
			proofbound.AbsSumUpper(part.RootGap, part.MeshBound))
		j.rootCollar = max(j.rootCollar, part.Collar)
	}
	return nil
}

// Payload audits the shell Shell built and proves its boundary, area and
// volume bounds. blankMesh is the blank's own verified mesh. The caller tags
// the two cap groups the record names and builds the body.
func (j *Join) Payload(ctx context.Context, blankMesh *topo.Mesh) (facetedrecord.Payload, error) {
	mesh := &j.mesh
	// In the angular disagreement between source and held tooth sectors, a
	// held ring point projects to the cylinder and then follows it to the
	// source crossing. The latter leg starts at the held root ray, not the
	// held crossing ray. Source cylinder points under a held tooth use the
	// separate source-to-held crossing shift. Hidden connector points use
	// their same-height chain directly.
	seamReach := proofbound.ProductUpper(2, j.innerInset)
	if j.allReach {
		seamReach = max(proofbound.AbsSumUpper(j.innerInset, j.angular),
			j.connectorReach, j.crossingShift)
	}
	collarSweep, ok := collarSweep(j.Teeth, j.Z0, j.Z1, j.rootCollar)
	if !ok {
		return facetedrecord.Payload{}, fmt.Errorf("%w: circular join lacks a cap-collar sweep bound", decaderr.ErrUnsupported)
	}
	bound := proofbound.AbsSumUpper(j.toothBound, seamReach, j.outerExcess, j.rootCollar, collarSweep)
	if proofbound.IsNonFinite(bound) || bound > 0.1 {
		return facetedrecord.Payload{}, fmt.Errorf("%w: circular join exceeds the 0.1 mm boundary proof: bound %g blank %g tooth %g inset %g round %g twist %g step %g", decaderr.ErrUnsupported, bound, blankMesh.Bound, j.toothBound, j.seamInset, j.round, j.twist, j.step)
	}
	if err := tessellation.OrientJoinedMesh(mesh.Verts, mesh.Tris); err != nil {
		return facetedrecord.Payload{}, err
	}
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewWorkBudget(ctx), mesh.Verts, mesh.Tris,
		loftmesh.MaxLoftAuditTriangles, AuditMaxScans); err != nil {
		return facetedrecord.Payload{}, err
	}
	for cap, outline := range j.outlines {
		z := j.Z0
		if cap == 1 {
			z = j.Z1
		}
		if !AuditCap(ctx, mesh.Verts, mesh.Tris, mesh.Src, outline, j.groups[cap+1], z, cap == 0) {
			if err := ctx.Err(); err != nil {
				return facetedrecord.Payload{}, err
			}
			return facetedrecord.Payload{}, fmt.Errorf("%w: circular join cap does not tile its recorded outline", decaderr.ErrUnsupported)
		}
		if !rootChordsInsideCap(ctx, mesh.Verts, outline, j.Teeth, cap) {
			if err := ctx.Err(); err != nil {
				return facetedrecord.Payload{}, err
			}
			return facetedrecord.Payload{}, fmt.Errorf("%w: circular join root chord leaves its cap", decaderr.ErrUnsupported)
		}
	}
	var sideMin2 *big.Rat
	for i, tri := range mesh.Tris {
		if mesh.Src[i] == j.groups[1] || mesh.Src[i] == j.groups[2] {
			continue
		}
		if q := TriangleMinRadius2(mesh.Verts, tri); sideMin2 == nil || q.Cmp(sideMin2) < 0 {
			sideMin2 = q
		}
	}
	minSideRadius := math.Inf(1)
	if sideMin2 != nil {
		minSideRadius = proofbound.RatSqrtDown(sideMin2)
	}
	if minSideRadius <= 0 || proofbound.IsNonFinite(minSideRadius) {
		return facetedrecord.Payload{}, fmt.Errorf("%w: circular join has no side clearance certificate", decaderr.ErrUnsupported)
	}
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, bound)
	if err != nil {
		return facetedrecord.Payload{}, err
	}
	// Outside this full radial band, the blank is either wholly occupied or
	// wholly absent, and the retained Loft facets have the same source roles.
	// Charge the entire band instead of assuming a facet correspondence at
	// the root seam. The source operands' own occupied-volume errors remain.
	if j.innerInset >= j.Radius {
		return facetedrecord.Payload{}, fmt.Errorf("%w: circular join reaches the axis", decaderr.ErrUnsupported)
	}
	bandWidth := proofbound.AbsSumUpper(j.innerInset, j.outerExcess)
	bandArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.ProductUpper(2, j.Radius), bandWidth),
		proofbound.ProductUpper(j.outerExcess, j.outerExcess))
	annulusArea := proofbound.ProductUpper(proofbound.RatFloatUp(proofbound.PiUpper), bandArea)
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(j.Z1), proofarith.FloatRat(j.Z0)))
	annulusVolume := proofbound.ProductUpper(annulusArea, height)
	volumeGap := proofbound.AbsSumUpper(blankMesh.VolSymDiff, j.toothVolume, annulusVolume)
	blankArea := j.Blank.Area
	areaSlack := proofbound.AbsSumUpper(area, blankArea.Value.Base(), blankArea.Bound.Base(), j.toothArea)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return facetedrecord.Payload{}, fmt.Errorf("%w: circular join has no finite volume proof", decaderr.ErrUnsupported)
	}
	vertexBound := make([]float64, len(mesh.Verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	toothBodies := make([]*topo.Body, len(j.Teeth))
	proofBodies := make([]*topo.Body, len(j.Teeth))
	axialBridge := make([]float64, len(j.Teeth))
	for i, part := range j.Teeth {
		toothBodies[i] = j.Originals[part.Operand]
		proofBodies[i] = j.Proofs[part.Operand]
		axialBridge[i] = part.Bridge
	}
	fp := facetedrecord.Payload{Verts: mesh.Verts, Tris: mesh.Tris, Src: mesh.Src,
		Groups: mesh.Groups, VertexBound: vertexBound, ContactAudited: true,
		MeshBound: bound, VolSymDiff: volumeGap, AreaSlack: areaSlack,
		DPair: facetedrecord.SourceDiameterUpper(mesh.Verts, bound),
		Xform: r3.Identity(), LoftCylinderJoin: &facetedrecord.LoftCylinderJoinRecord{
			Blank: j.Blank, Teeth: toothBodies, ProofTeeth: proofBodies,
			AxialBridge: axialBridge, CapOutlines: j.outlines,
			CapGroups:     [2]int{j.groups[1], j.groups[2]},
			MinSideRadius: minSideRadius, Radius: j.Radius, Z0: j.Z0, Z1: j.Z1,
			InnerInset: j.innerInset, OuterExcess: j.outerExcess, RootCollar: j.rootCollar,
		}}
	if proofbound.IsNonFinite(fp.DPair) {
		return facetedrecord.Payload{}, fmt.Errorf("%w: circular join has no finite diameter proof", decaderr.ErrUnsupported)
	}
	return fp, nil
}

// collarSweep bounds the lateral travel of a cap-collar point. Source root
// and connector rulings move linearly between the cap sections. A
// cap-collar point is at most collar/height of its whole XY travel from the
// matching cap point. Source wall positions lie within the held wall's
// matched Loft bound, so the paired held vertices give a radius ceiling.
func collarSweep(parts []Tooth, z0, z1, collar float64) (float64, bool) {
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
		for _, p := range part.Loft.Verts {
			x, y := proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)
			radius := proofbound.RatSqrtUp(new(big.Rat).Add(
				new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y)))
			toothMaximum = max(toothMaximum, radius)
		}
		maximum = max(maximum, proofbound.AbsSumUpper(toothMaximum, part.Loft.Proof.FacetDeparture))
	}
	ratio := proofbound.RatFloatUp(new(big.Rat).Quo(proofarith.FloatRat(collar), height))
	sweep := proofbound.ProductUpper(proofbound.ProductUpper(2, maximum), ratio)
	return sweep, !proofbound.IsNonFinite(sweep)
}

// rootChordsInsideCap checks the omitted held root chords against one
// audited cap. The cap audit establishes a simple filled polygon. Each
// omitted held root chord stays in that polygon if one interior point is
// strictly inside and the chord has no interior contact with its outline.
// Matched source root arcs then use their authenticated Loft departure to
// reach the cap.
func rootChordsInsideCap(ctx context.Context, verts []r3.Vec,
	outline []int, parts []Tooth, capIndex int) bool {
	if capIndex < 0 || capIndex > 1 || len(outline) < 3 {
		return false
	}
	points := make([]XY, len(outline))
	poly := make([]meshbool.Xp2, len(outline))
	for i, index := range outline {
		if index < 0 || index >= len(verts) {
			return false
		}
		points[i] = ExactXY(verts[index])
		poly[i] = meshbool.NewXP2(points[i].X, points[i].Y)
	}
	same := func(a, b XY) bool {
		return a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0
	}
	budget := proofbound.NewWorkBudget(ctx)
	for _, part := range parts {
		lp := part.Loft
		n := len(lp.Verts) / 2
		if n < 2 || part.RootStart < 0 || part.RootStart >= n-1 {
			return false
		}
		rootA := ExactXY(lp.Verts[capIndex*n+part.RootStart])
		rootB := ExactXY(lp.Verts[capIndex*n])
		for j := part.RootStart; j < n; j++ {
			if ctx.Err() != nil {
				return false
			}
			a := ExactXY(lp.Verts[capIndex*n+j])
			b := ExactXY(lp.Verts[capIndex*n+(j+1)%n])
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
				if !EdgesMeet(a, b, c, d) {
					continue
				}
				// A non-collinear outline edge may touch only a shared root
				// endpoint. All other contact, including overlap, refuses.
				shared := (same(a, rootA) || same(a, rootB)) && (same(a, c) || same(a, d)) ||
					(same(b, rootA) || same(b, rootB)) && (same(b, c) || same(b, d))
				if !shared || Orient(a, b, c).Sign() == 0 &&
					Orient(a, b, d).Sign() == 0 {
					return false
				}
			}
		}
	}
	return true
}
