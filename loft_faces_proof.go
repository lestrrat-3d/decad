package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// The rational source maps are kept here rather than reconstructed from
// public cap edges. A rounded FacetedCurve does not contain a fitted flank.
type loftFaceRatVec [3]*big.Rat
type loftFaceRatMat [3][3]*big.Rat

type loftFaceAffine struct {
	a loftFaceRatMat
	b loftFaceRatVec
}

type loftFaceMap struct {
	constant   loftFaceRatVec
	q0, qu, qv loftFaceRatVec
	g0, gu, gv *big.Rat
	stretch    float64
}

type loftFaceInterval struct{ lo, hi *big.Rat }

func lfZero() *big.Rat             { return new(big.Rat) }
func lfOne() *big.Rat              { return big.NewRat(1, 1) }
func lfF(v float64) *big.Rat       { return ratOfPointPlane(v) }
func lfAdd(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
func lfSub(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
func lfMul(a, b *big.Rat) *big.Rat { return new(big.Rat).Mul(a, b) }
func lfDiv(a, b *big.Rat) *big.Rat { return new(big.Rat).Quo(a, b) }
func lfAbs(a *big.Rat) *big.Rat    { return new(big.Rat).Abs(a) }

func lfVec(v r3.Vec) loftFaceRatVec { return loftFaceRatVec{lfF(v.X), lfF(v.Y), lfF(v.Z)} }

func lfVAdd(a, b loftFaceRatVec) loftFaceRatVec {
	return loftFaceRatVec{lfAdd(a[0], b[0]), lfAdd(a[1], b[1]), lfAdd(a[2], b[2])}
}

func lfVSub(a, b loftFaceRatVec) loftFaceRatVec {
	return loftFaceRatVec{lfSub(a[0], b[0]), lfSub(a[1], b[1]), lfSub(a[2], b[2])}
}

func lfVScale(a loftFaceRatVec, s *big.Rat) loftFaceRatVec {
	return loftFaceRatVec{lfMul(a[0], s), lfMul(a[1], s), lfMul(a[2], s)}
}

func lfVDot(a, b loftFaceRatVec) *big.Rat {
	return lfAdd(lfAdd(lfMul(a[0], b[0]), lfMul(a[1], b[1])), lfMul(a[2], b[2]))
}

func lfVCross(a, b loftFaceRatVec) loftFaceRatVec {
	return loftFaceRatVec{
		lfSub(lfMul(a[1], b[2]), lfMul(a[2], b[1])),
		lfSub(lfMul(a[2], b[0]), lfMul(a[0], b[2])),
		lfSub(lfMul(a[0], b[1]), lfMul(a[1], b[0])),
	}
}

func lfMatVec(m loftFaceRatMat, v loftFaceRatVec) loftFaceRatVec {
	var out loftFaceRatVec
	for i := range out {
		out[i] = lfZero()
		for j := range v {
			out[i] = lfAdd(out[i], lfMul(m[i][j], v[j]))
		}
	}
	return out
}

func lfMatMul(a, b loftFaceRatMat) loftFaceRatMat {
	var out loftFaceRatMat
	for i := range out {
		for j := range out[i] {
			out[i][j] = lfZero()
			for k := range a[i] {
				out[i][j] = lfAdd(out[i][j], lfMul(a[i][k], b[k][j]))
			}
		}
	}
	return out
}

func lfIdentity() loftFaceAffine {
	var out loftFaceAffine
	for i := range out.a {
		out.b[i] = lfZero()
		for j := range out.a[i] {
			out.a[i][j] = lfZero()
		}
		out.a[i][i] = lfOne()
	}
	return out
}

func lfMotion(t r3.Transform) loftFaceAffine {
	basis := t.Basis()
	cols := [3]r3.Vec{basis.EX, basis.EY, basis.EZ}
	out := loftFaceAffine{b: lfVec(t.Translation())}
	for j, col := range cols {
		v := lfVec(col)
		for i := range v {
			out.a[i][j] = v[i]
		}
	}
	return out
}

func lfScale(anchor r3.Vec, factor float64) loftFaceAffine {
	out := lfIdentity()
	for i, v := range lfVec(anchor) {
		out.a[i][i] = lfF(factor)
		out.b[i] = lfMul(lfSub(lfOne(), lfF(factor)), v)
	}
	return out
}

func lfCompose(prior, next loftFaceAffine) loftFaceAffine {
	return loftFaceAffine{a: lfMatMul(next.a, prior.a),
		b: lfVAdd(lfMatVec(next.a, prior.b), next.b)}
}

func lfSimilarity(section loftFaceSection) (loftFaceAffine, error) {
	out := lfIdentity()
	if section.mesh.pointSimilarity != nil {
		for _, step := range section.mesh.pointSimilarity.steps {
			if step.factor == 0 {
				if !step.motion.IsValid() {
					return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid rigid step", ErrUnsupported)
				}
				out = lfCompose(out, lfMotion(step.motion))
			} else {
				if !proofbound.FiniteVec(step.anchor) || !proofbound.FiniteVec(r3.Vec{X: step.factor}) ||
					step.factor <= 0 {
					return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid scale step", ErrUnsupported)
				}
				out = lfCompose(out, lfScale(step.anchor, step.factor))
			}
		}
	} else if section.mesh.xform != r3.Identity() {
		if !section.mesh.xform.IsValid() {
			return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid placement", ErrUnsupported)
		}
		out = lfCompose(out, lfMotion(section.mesh.xform))
	}
	return out, nil
}

func lfAffineStretch(a loftFaceRatMat) float64 {
	squares := 0.0
	for i := range a {
		for j := range a[i] {
			v := lfRatUp(lfAbs(a[i][j]))
			squares = proofbound.AbsSumUpper(squares, proofbound.ProductUpper(v, v))
		}
	}
	return math.Nextafter(math.Sqrt(squares), math.Inf(1))
}

func lfCapMap(section loftFaceSection, source *pointSectionRecord) (loftFaceMap, error) {
	affine, err := lfSimilarity(section)
	if err != nil {
		return loftFaceMap{}, err
	}
	apex := lfVec(source.apex)
	origin := lfVec(source.plane.Origin)
	uAxis := lfVec(source.plane.U)
	vAxis := lfVec(source.plane.V)
	var n loftFaceRatVec
	var d *big.Rat
	if section.limit == nil {
		n = loftFaceRatVec{lfZero(), lfZero(), lfZero()}
		d = lfOne()
	} else {
		n = loftFaceRatVec{section.limit.nx, section.limit.ny, lfZero()}
		d = section.limit.d
	}
	g0, gu, gv := lfVDot(n, lfVSub(origin, apex)), lfVDot(n, uAxis), lfVDot(n, vAxis)
	if section.limit == nil {
		g0 = lfOne()
	}
	q0 := lfVScale(lfMatVec(affine.a, lfVSub(origin, apex)), d)
	qu := lfVScale(lfMatVec(affine.a, uAxis), d)
	qv := lfVScale(lfMatVec(affine.a, vAxis), d)
	stretch := lfAffineStretch(affine.a)
	if section.limit != nil {
		stretch = proofbound.ProductUpper(stretch, section.limit.mapStretch)
	}
	if proofbound.IsNonFinite(stretch) {
		return loftFaceMap{}, fmt.Errorf("%w: loft cap source stretch is not finite", ErrUnsupported)
	}
	return loftFaceMap{constant: lfVAdd(lfMatVec(affine.a, apex), affine.b),
		q0: q0, qu: qu, qv: qv, g0: g0, gu: gu, gv: gv, stretch: stretch}, nil
}

func (m loftFaceMap) denominator(u, v *big.Rat) *big.Rat {
	return lfAdd(lfAdd(m.g0, lfMul(m.gu, u)), lfMul(m.gv, v))
}

func (m loftFaceMap) numerator(u, v *big.Rat) loftFaceRatVec {
	return lfVAdd(lfVAdd(m.q0, lfVScale(m.qu, u)), lfVScale(m.qv, v))
}

func (m loftFaceMap) at(u, v *big.Rat) loftFaceRatVec {
	g := m.denominator(u, v)
	return lfVAdd(m.constant, lfVScale(m.numerator(u, v), lfDiv(lfOne(), g)))
}

func (m loftFaceMap) deriv(u, v *big.Rat, axis int) loftFaceRatVec {
	g := m.denominator(u, v)
	q := m.numerator(u, v)
	qAxis, gAxis := m.qu, m.gu
	if axis == 1 {
		qAxis, gAxis = m.qv, m.gv
	}
	return lfVScale(lfVSub(lfVScale(qAxis, g), lfVScale(q, gAxis)),
		lfDiv(lfOne(), lfMul(g, g)))
}

func lfRatUp(q *big.Rat) float64 {
	f, _ := q.Float64()
	if !proofbound.IsNonFinite(f) && lfF(f).Cmp(q) < 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f
}

func lfIntervalHull(a, b loftFaceInterval) loftFaceInterval {
	out := a
	if b.lo.Cmp(out.lo) < 0 {
		out.lo = b.lo
	}
	if b.hi.Cmp(out.hi) > 0 {
		out.hi = b.hi
	}
	return out
}

func lfIntervalSub(a, b loftFaceInterval) loftFaceInterval {
	return loftFaceInterval{lfSub(a.lo, b.hi), lfSub(a.hi, b.lo)}
}

func lfIntervalAbs(a loftFaceInterval, center *big.Rat) *big.Rat {
	lo, hi := lfAbs(lfSub(a.lo, center)), lfAbs(lfSub(a.hi, center))
	if lo.Cmp(hi) > 0 {
		return lo
	}
	return hi
}

func lfTileCorners(u0, u1, v0, v1 *big.Rat) [4][2]*big.Rat {
	return [4][2]*big.Rat{{u0, v0}, {u0, v1}, {u1, v0}, {u1, v1}}
}

func lfScalarRange(m loftFaceMap, row loftFaceRatVec, corners [4][2]*big.Rat) loftFaceInterval {
	var out loftFaceInterval
	for i, corner := range corners {
		value := lfVDot(row, m.at(corner[0], corner[1]))
		if i == 0 {
			out = loftFaceInterval{value, value}
			continue
		}
		if value.Cmp(out.lo) < 0 {
			out.lo = value
		}
		if value.Cmp(out.hi) > 0 {
			out.hi = value
		}
	}
	return out
}

func lfDerivRange(m loftFaceMap, row loftFaceRatVec, axis int,
	corners [4][2]*big.Rat) (loftFaceInterval, error) {
	var num, den loftFaceInterval
	for i, corner := range corners {
		g := m.denominator(corner[0], corner[1])
		if g.Sign() <= 0 {
			return loftFaceInterval{}, fmt.Errorf("%w: loft cap ray denominator is not positive", ErrUnsupported)
		}
		q := m.numerator(corner[0], corner[1])
		qAxis, gAxis := m.qu, m.gu
		if axis == 1 {
			qAxis, gAxis = m.qv, m.gv
		}
		n := lfVDot(row, lfVSub(lfVScale(qAxis, g), lfVScale(q, gAxis)))
		if i == 0 {
			num = loftFaceInterval{n, n}
			den = loftFaceInterval{g, g}
			continue
		}
		if n.Cmp(num.lo) < 0 {
			num.lo = n
		}
		if n.Cmp(num.hi) > 0 {
			num.hi = n
		}
		if g.Cmp(den.lo) < 0 {
			den.lo = g
		}
		if g.Cmp(den.hi) > 0 {
			den.hi = g
		}
	}
	lo2, hi2 := lfMul(den.lo, den.lo), lfMul(den.hi, den.hi)
	values := [4]*big.Rat{lfDiv(num.lo, lo2), lfDiv(num.lo, hi2),
		lfDiv(num.hi, lo2), lfDiv(num.hi, hi2)}
	out := loftFaceInterval{values[0], values[0]}
	for _, v := range values[1:] {
		if v.Cmp(out.lo) < 0 {
			out.lo = v
		}
		if v.Cmp(out.hi) > 0 {
			out.hi = v
		}
	}
	return out, nil
}

func lfInverseRows(c0, c1, c2 loftFaceRatVec) ([3]loftFaceRatVec, *big.Rat, error) {
	det := lfVDot(c0, lfVCross(c1, c2))
	if det.Sign() == 0 {
		return [3]loftFaceRatVec{}, nil,
			fmt.Errorf("%w: loft band source map is singular", ErrUnsupported)
	}
	return [3]loftFaceRatVec{
		lfVScale(lfVCross(c1, c2), lfDiv(lfOne(), det)),
		lfVScale(lfVCross(c2, c0), lfDiv(lfOne(), det)),
		lfVScale(lfVCross(c0, c1), lfDiv(lfOne(), det)),
	}, det, nil
}

func lfSourceBox(source *pointSectionRecord) ([4]*big.Rat, error) {
	var box [4]*big.Rat
	for i, v := range source.sourceBox {
		if proofbound.IsNonFinite(v) {
			return box,
				fmt.Errorf("%w: loft source rectangle is not finite", ErrUnsupported)
		}
		box[i] = lfF(v)
	}
	if box[0].Cmp(box[1]) >= 0 || box[2].Cmp(box[3]) >= 0 {
		return box, fmt.Errorf("%w: loft source rectangle has no area", ErrUnsupported)
	}
	return box, nil
}

func lfBoxCorners(box [4]*big.Rat) [4][2]*big.Rat {
	return lfTileCorners(box[0], box[1], box[2], box[3])
}

func lfPositiveDenominator(m loftFaceMap, box [4]*big.Rat) error {
	for _, corner := range lfBoxCorners(box) {
		if m.denominator(corner[0], corner[1]).Sign() <= 0 {
			return fmt.Errorf("%w: loft cap ray denominator crosses its source region", ErrUnsupported)
		}
	}
	return nil
}

func boolInt(ok bool) int {
	if ok {
		return 1
	}
	return 0
}

// lfPlaneOrder proves that an exact intermediate cap separates every earlier
// and later cap. Since each band is a convex interpolation of its end caps,
// this also separates nonadjacent bands without a sampled crossing claim.
func lfPlaneOrder(maps []loftFaceMap, box [4]*big.Rat) error {
	corners := lfBoxCorners(box)
	for middle := 1; middle+1 < len(maps); middle++ {
		p0 := maps[middle].at(corners[0][0], corners[0][1])
		p1 := maps[middle].at(corners[2][0], corners[2][1])
		p2 := maps[middle].at(corners[1][0], corners[1][1])
		normal := lfVCross(lfVSub(p1, p0), lfVSub(p2, p0))
		if normal[0].Sign() == 0 && normal[1].Sign() == 0 && normal[2].Sign() == 0 {
			return fmt.Errorf("%w: loft cap %d has no exact supporting plane", ErrUnsupported, middle)
		}
		previousSign := 0
		for section, capMap := range maps {
			if section == middle {
				continue
			}
			for _, corner := range corners {
				sign := lfVDot(normal, lfVSub(capMap.at(corner[0], corner[1]), p0)).Sign()
				if sign == 0 {
					return fmt.Errorf("%w: loft cap %d meets section %d", ErrUnsupported, middle, section)
				}
				if section < middle {
					if previousSign == 0 {
						previousSign = sign
					}
					if sign != previousSign {
						return fmt.Errorf("%w: loft cap %d does not separate earlier sections", ErrUnsupported, middle)
					}
				} else if sign == previousSign {
					return fmt.Errorf("%w: loft cap %d does not separate later sections", ErrUnsupported, middle)
				}
			}
		}
	}
	return nil
}

type loftFaceEmbedding struct {
	maps        []loftFaceMap
	jacobianSum float64
	maximumNorm float64
	sign        int
}

type loftFaceCapBounds struct {
	stationRound        float64
	projectiveDeparture []float64
	polygonSpeed        []float64
	sourceSpeed         []float64
	sourceDeparture     []float64
}

func lfStationUV(point Point2) (*big.Rat, *big.Rat) { return lfF(point.U), lfF(point.V) }

func lfCapBounds(ctx context.Context, capMap loftFaceMap, section loftFaceSection,
	source *pointSectionRecord) (loftFaceCapBounds, error) {
	n := len(source.sourceStation)
	var out loftFaceCapBounds
	if len(section.ring) != n || len(source.sourceArcUpper) != n || len(source.sourceMatched) != n {
		return out, fmt.Errorf("%w: loft source station proof is incomplete", ErrUnsupported)
	}
	out.projectiveDeparture = make([]float64, n)
	out.polygonSpeed = make([]float64, n)
	out.sourceSpeed = make([]float64, n)
	out.sourceDeparture = make([]float64, n)
	exact := make([]loftFaceRatVec, n)
	den := make([]*big.Rat, n)
	for j, station := range source.sourceStation {
		if j%256 == 0 {
			if err := ctx.Err(); err != nil {
				return out, err
			}
		}
		u, v := lfStationUV(station)
		den[j] = capMap.denominator(u, v)
		if den[j].Sign() <= 0 {
			return out,
				fmt.Errorf("%w: loft station ray denominator is not positive", ErrUnsupported)
		}
		exact[j] = capMap.at(u, v)
		held := [3]float64{section.ring[j].X, section.ring[j].Y, section.ring[j].Z}
		perCoord := 0.0
		for axis := range held {
			perCoord = max(perCoord, proofbound.RatAbsDiff(exact[j][axis], held[axis]))
		}
		out.stationRound = max(out.stationRound, proofbound.Radius3D(perCoord))
	}
	for j := range n {
		jn := (j + 1) % n
		chord := lfVSub(exact[jn], exact[j])
		lengthL1 := lfAdd(lfAdd(lfAbs(chord[0]), lfAbs(chord[1])), lfAbs(chord[2]))
		minDen, maxDen := den[j], den[jn]
		if minDen.Cmp(maxDen) > 0 {
			minDen, maxDen = maxDen, minDen
		}
		shift := lfDiv(lfMul(lfAbs(lfSub(den[jn], den[j])), lengthL1),
			lfMul(big.NewRat(4, 1), minDen))
		speed := lfDiv(lfMul(maxDen, lengthL1), minDen)
		out.projectiveDeparture[j] = lfRatUp(shift)
		out.polygonSpeed[j] = lfRatUp(speed)
		out.sourceSpeed[j] = proofbound.ProductUpper(source.sourceArcUpper[j], capMap.stretch)
		out.sourceDeparture[j] = proofbound.ProductUpper(
			max(source.sourceMatched[j], section.slice.base.meshBound), capMap.stretch)
		if proofbound.IsNonFinite(out.projectiveDeparture[j]) ||
			proofbound.IsNonFinite(out.polygonSpeed[j]) ||
			proofbound.IsNonFinite(out.sourceSpeed[j]) ||
			proofbound.IsNonFinite(out.sourceDeparture[j]) {
			return out, fmt.Errorf("%w: loft cap cell has no finite source bound", ErrUnsupported)
		}
	}
	if proofbound.IsNonFinite(out.stationRound) {
		return out, fmt.Errorf("%w: loft cap station rounding is not finite", ErrUnsupported)
	}
	return out, nil
}

// proveLoftFaceBounds composes source-region, projective parameter, ruled
// wall, bilinear twist and held-station rounding bounds without replacing
// the authenticated fitted source by sampled cap edges.
func proveLoftFaceBounds(ctx context.Context, sections []loftFaceSection,
	source *pointSectionRecord, embedding loftFaceEmbedding,
	mesh facetedPayload) (facetedPayload, error) {
	proofs := make([]loftFaceCapBounds, len(sections))
	maxRound := 0.0
	for i, section := range sections {
		var err error
		proofs[i], err = lfCapBounds(ctx, embedding.maps[i], section, source)
		if err != nil {
			return facetedPayload{}, fmt.Errorf("loft cap %d: %w", i, err)
		}
		maxRound = max(maxRound, proofs[i].stationRound)
	}
	meshBound, wallLeg, twistVolume, trueWallArea := 0.0, 0.0, 0.0, 0.0
	n := len(source.sourceStation)
	for band := 0; band+1 < len(sections); band++ {
		lo, hi := proofs[band], proofs[band+1]
		for j := range n {
			if j%256 == 0 {
				if err := ctx.Err(); err != nil {
					return facetedPayload{}, err
				}
			}
			jn := (j + 1) % n
			vLo, vHi := sections[band].ring[j], sections[band].ring[jn]
			wLo, wHi := sections[band+1].ring[j], sections[band+1].ring[jn]
			polygonMatch := max(proofbound.AbsSumUpper(lo.projectiveDeparture[j], lo.stationRound),
				proofbound.AbsSumUpper(hi.projectiveDeparture[j], hi.stationRound))
			wallArea := proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi,
				lo.polygonSpeed[j], hi.polygonSpeed[j], polygonMatch)
			wallLeg = proofbound.AbsSumUpper(wallLeg, proofbound.ProductUpper(polygonMatch, wallArea))
			twistVolume = proofbound.AbsSumUpper(twistVolume,
				proofbound.CellTwistVolumeAllow(vLo, vHi, wLo, wHi))
			smoothMatch := max(proofbound.AbsSumUpper(lo.sourceDeparture[j],
				lo.projectiveDeparture[j], lo.stationRound),
				proofbound.AbsSumUpper(hi.sourceDeparture[j],
					hi.projectiveDeparture[j], hi.stationRound))
			meshBound = max(meshBound, proofbound.AbsSumUpper(smoothMatch,
				proofbound.CellTwistOffsetUpper(vLo, vHi, wLo, wHi)))
			trueWallArea = proofbound.AbsSumUpper(trueWallArea,
				proofbound.CellChordCurveAreaUpper(vLo, vHi, wLo, wHi,
					lo.sourceSpeed[j], hi.sourceSpeed[j], smoothMatch))
		}
	}
	for _, proof := range []loftFaceCapBounds{proofs[0], proofs[len(proofs)-1]} {
		for j := range proof.sourceDeparture {
			meshBound = max(meshBound, proofbound.AbsSumUpper(proof.sourceDeparture[j],
				proof.projectiveDeparture[j], proof.stationRound))
		}
	}
	// This whole-mesh sweep charges the exact source-map station coordinates
	// against the stored float vertices, including both end caps. The wall
	// matched-departure leg may also charge this rounding; duplication is safe.
	roundArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, maxRound)
	if err != nil {
		return facetedPayload{}, err
	}
	roundSweep := proofbound.SweptVolumeAllow(maxRound, roundArea)
	sourceRegion := proofbound.ProductUpper(source.sourceAreaDelta, embedding.jacobianSum)
	volumeGap := proofbound.AbsSumUpper(sourceRegion, wallLeg, twistVolume, roundSweep)
	base := sections[0].slice.base
	baseArea, err := proofbound.PerturbedAreaUpperContext(ctx, base.verts, base.tris, base.meshBound)
	if err != nil {
		return facetedPayload{}, err
	}
	baseArea = proofbound.AbsSumUpper(baseArea, base.areaSlack)
	endCapArea := 0.0
	for _, i := range []int{0, len(sections) - 1} {
		stretch2 := proofbound.ProductUpper(embedding.maps[i].stretch, embedding.maps[i].stretch)
		endCapArea = proofbound.AbsSumUpper(endCapArea, proofbound.ProductUpper(baseArea, stretch2))
	}
	heldArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, 0)
	if err != nil {
		return facetedPayload{}, err
	}
	areaSlack := proofbound.AbsSumUpper(trueWallArea, endCapArea, heldArea)
	if proofbound.IsNonFinite(meshBound) || proofbound.IsNonFinite(volumeGap) ||
		proofbound.IsNonFinite(areaSlack) {
		return facetedPayload{}, fmt.Errorf("%w: face-fed loft proof has no finite bounds", ErrUnsupported)
	}
	mesh.vertexBound = make([]float64, len(mesh.verts))
	for i := range mesh.vertexBound {
		mesh.vertexBound[i] = meshBound
	}
	mesh.meshBound = meshBound
	mesh.volSymDiff = volumeGap
	mesh.areaSlack = areaSlack
	mesh.dPair = facetedSourceDiameterUpper(mesh.verts, meshBound)
	mesh.contactAudited = true
	return mesh, nil
}

func proveLoftFaceEmbedding(ctx context.Context, sections []loftFaceSection,
	source *pointSectionRecord) (loftFaceEmbedding, error) {
	var proof loftFaceEmbedding
	box, err := lfSourceBox(source)
	if err != nil {
		return proof, err
	}
	proof.maps = make([]loftFaceMap, len(sections))
	for i, section := range sections {
		if err := ctx.Err(); err != nil {
			return proof, err
		}
		proof.maps[i], err = lfCapMap(section, source)
		if err != nil {
			return proof, fmt.Errorf("loft section %d: %w", i, err)
		}
		if err := lfPositiveDenominator(proof.maps[i], box); err != nil {
			return proof, fmt.Errorf("loft section %d: %w", i, err)
		}
	}
	if err := lfPlaneOrder(proof.maps, box); err != nil {
		return proof, err
	}
	for band := 0; band+1 < len(proof.maps); band++ {
		if err := ctx.Err(); err != nil {
			return proof, err
		}
		rho, jac, sign, gateErr := lfBandGate(ctx, proof.maps[band], proof.maps[band+1], box)
		if gateErr != nil {
			return proof, fmt.Errorf("loft band %d: %w", band, gateErr)
		}
		if proof.sign != 0 && proof.sign != sign {
			return proof, fmt.Errorf("%w: loft bands reverse their source orientation", ErrUnsupported)
		}
		proof.sign = sign
		proof.maximumNorm = max(proof.maximumNorm, rho)
		proof.jacobianSum = proofbound.AbsSumUpper(proof.jacobianSum, jac)
	}
	return proof, nil
}

func orientLoftFaces(mesh *facetedPayload, base facetedPayload,
	source *pointSectionRecord, jacobianSign int) error {
	n := len(source.sourceStation)
	if n < 3 || len(base.tris) <= n || jacobianSign == 0 {
		return fmt.Errorf("%w: loft source cap has no orientation", ErrUnsupported)
	}
	polygonSign := 0
	for _, tri := range base.tris[n:] {
		if tri[0] <= 0 || tri[1] <= 0 || tri[2] <= 0 ||
			tri[0] > n || tri[1] > n || tri[2] > n {
			return fmt.Errorf("%w: loft source cap has invalid station indices", ErrUnsupported)
		}
		a, b, c := source.sourceStation[tri[0]-1], source.sourceStation[tri[1]-1],
			source.sourceStation[tri[2]-1]
		cross := lfSub(lfMul(lfSub(lfF(b.U), lfF(a.U)), lfSub(lfF(c.V), lfF(a.V))),
			lfMul(lfSub(lfF(b.V), lfF(a.V)), lfSub(lfF(c.U), lfF(a.U))))
		sign := cross.Sign()
		if sign == 0 || (polygonSign != 0 && sign != polygonSign) {
			return fmt.Errorf("%w: loft source cap triangulation changes orientation", ErrUnsupported)
		}
		polygonSign = sign
	}
	if polygonSign != jacobianSign {
		for i := range mesh.tris {
			mesh.tris[i][1], mesh.tris[i][2] = mesh.tris[i][2], mesh.tris[i][1]
		}
		mesh.groups[len(mesh.groups)-2].reversed = false
		mesh.groups[len(mesh.groups)-1].reversed = true
	}
	return nil
}
