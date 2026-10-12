package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

type obliqueIV = proofbound.RatInterval

type obliqueIVPoint struct{ x, y, z obliqueIV }

func obliqueRatDot(a, b r3.Vec) *big.Rat {
	out := new(big.Rat)
	for _, pair := range [3][2]float64{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
		out.Add(out, new(big.Rat).Mul(proofarith.FloatRat(pair[0]), proofarith.FloatRat(pair[1])))
	}
	return out
}

func obliqueBasisAdmitted(rec *obliqueBossRecord) bool {
	if rec.axis.Z <= 0.94 {
		return false
	}
	tiltSquared := obliqueRatDot(r3.Vec{X: rec.axis.X, Y: rec.axis.Y},
		r3.Vec{X: rec.axis.X, Y: rec.axis.Y})
	tiltLimit := proofarith.FloatRat(0.349)
	if tiltSquared.Cmp(new(big.Rat).Mul(tiltLimit, tiltLimit)) >= 0 {
		return false
	}
	lo, hi := proofarith.FloatRat(0.999), proofarith.FloatRat(1.009)
	lo.Mul(lo, lo)
	hi.Mul(hi, hi)
	for _, basis := range []r3.Vec{rec.axis, rec.ex, rec.ey} {
		square := obliqueRatDot(basis, basis)
		if square.Cmp(lo) <= 0 || square.Cmp(hi) >= 0 {
			return false
		}
	}
	dotLimit := proofarith.FloatRat(0.0009)
	for _, pair := range [][2]r3.Vec{{rec.axis, rec.ex}, {rec.axis, rec.ey}, {rec.ex, rec.ey}} {
		if new(big.Rat).Abs(obliqueRatDot(pair[0], pair[1])).Cmp(dotLimit) >= 0 {
			return false
		}
	}
	return true
}

func obliqueBossDiameterBound(rec *obliqueBossRecord, boundaryBound float64) float64 {
	w := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.rect.u1), proofarith.FloatRat(rec.rect.u0)))
	h := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.rect.v1), proofarith.FloatRat(rec.rect.v0)))
	plateHeight := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.top), proofarith.FloatRat(rec.bottom)))
	w = proofbound.AbsSumUpper(w, proofbound.ProductUpper(2, boundaryBound))
	h = proofbound.AbsSumUpper(h, proofbound.ProductUpper(2, boundaryBound))
	z := proofbound.AbsSumUpper(plateHeight, rec.length, rec.radius,
		proofbound.ProductUpper(2, boundaryBound))
	return proofbound.Radius3D(max(w, h, z))
}

func obliqueIVOf(f float64) obliqueIV {
	return proofbound.PointInterval(proofarith.FloatRat(f))
}

func obliqueZeroPoint() obliqueIVPoint {
	z := obliqueIVOf(0)
	return obliqueIVPoint{z, z, z}
}

func obliqueIVVec(v r3.Vec) obliqueIVPoint {
	return obliqueIVPoint{obliqueIVOf(v.X), obliqueIVOf(v.Y), obliqueIVOf(v.Z)}
}

func (p obliqueIVPoint) add(q obliqueIVPoint) obliqueIVPoint {
	return obliqueIVPoint{
		proofbound.IntervalAdd(p.x, q.x),
		proofbound.IntervalAdd(p.y, q.y),
		proofbound.IntervalAdd(p.z, q.z),
	}
}

func (p obliqueIVPoint) sub(q obliqueIVPoint) obliqueIVPoint {
	return obliqueIVPoint{
		proofbound.IntervalSub(p.x, q.x),
		proofbound.IntervalSub(p.y, q.y),
		proofbound.IntervalSub(p.z, q.z),
	}
}

func (p obliqueIVPoint) mul(q obliqueIV) obliqueIVPoint {
	return obliqueIVPoint{
		proofbound.IntervalMul(p.x, q),
		proofbound.IntervalMul(p.y, q),
		proofbound.IntervalMul(p.z, q),
	}
}

func (p obliqueIVPoint) norm() (obliqueIV, bool) {
	squares := proofbound.IntervalAdd(proofbound.IntervalSquare(p.x), proofbound.IntervalSquare(p.y))
	squares = proofbound.IntervalAdd(squares, proofbound.IntervalSquare(p.z))
	return proofbound.SqrtInterval(squares)
}

func (p obliqueIVPoint) normalized() (obliqueIVPoint, bool) {
	norm, ok := p.norm()
	if !ok || norm.Lo.Sign() <= 0 {
		return obliqueIVPoint{}, false
	}
	x, ok := proofbound.IntervalQuo(p.x, norm)
	if !ok {
		return obliqueIVPoint{}, false
	}
	y, ok := proofbound.IntervalQuo(p.y, norm)
	if !ok {
		return obliqueIVPoint{}, false
	}
	z, ok := proofbound.IntervalQuo(p.z, norm)
	return obliqueIVPoint{x, y, z}, ok
}

func (p obliqueIVPoint) error(held r3.Vec) float64 {
	return proofbound.Radius3D(max(
		proofbound.IntervalFloatError(p.x, held.X),
		proofbound.IntervalFloatError(p.y, held.Y),
		proofbound.IntervalFloatError(p.z, held.Z)))
}

// obliqueSampleError encloses the exact value of every sampled ideal patch
// point. Trigonometry and normal lengths use rational outward intervals;
// no assumed libm ulp count enters the published boundary bound.
func obliqueSampleError(rec *obliqueBossRecord, radius float64,
	u float64, held []r3.Vec) (float64, error) {
	if len(held) != 5+obliqueBossBlendCells-1 {
		return 0, fmt.Errorf("%w: an oblique sample has the wrong vertex count", ErrBooleanFailed)
	}
	sn, cs, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(u))
	if !ok {
		return 0, fmt.Errorf("%w: an oblique sample has no trigonometric enclosure", ErrUnsupported)
	}
	d := obliqueIVVec(rec.ex).mul(cs).add(obliqueIVVec(rec.ey).mul(sn))
	rPlus := proofbound.IntervalAdd(obliqueIVOf(rec.radius), obliqueIVOf(radius))
	topPlus := proofbound.IntervalAdd(obliqueIVOf(rec.top), obliqueIVOf(radius))
	tNumerator := proofbound.IntervalSub(
		proofbound.IntervalSub(topPlus, obliqueIVOf(rec.base.Z)),
		proofbound.IntervalMul(rPlus, d.z))
	t, ok := proofbound.IntervalQuo(tNumerator, obliqueIVOf(rec.axis.Z))
	if !ok {
		return 0, fmt.Errorf("%w: an oblique sample has no axial enclosure", ErrUnsupported)
	}
	c := obliqueIVVec(rec.base).add(d.mul(rPlus)).add(obliqueIVVec(rec.axis).mul(t))
	vertical := obliqueZeroPoint()
	vertical.z = obliqueIVOf(radius)
	p := c.sub(vertical)
	s := c.sub(d.mul(obliqueIVOf(radius)))
	far := obliqueIVVec(rec.base).add(d.mul(obliqueIVOf(rec.radius))).
		add(obliqueIVVec(rec.axis).mul(obliqueIVOf(rec.length)))
	worst := max(p.error(held[2]), s.error(held[3]), far.error(held[4]))
	for j := 1; j < obliqueBossBlendCells; j++ {
		v := big.NewRat(int64(j), obliqueBossBlendCells)
		q := obliqueZeroPoint()
		q.z = proofbound.PointInterval(new(big.Rat).Sub(big.NewRat(1, 1), v))
		q = q.add(d.mul(proofbound.PointInterval(v)))
		n, ok := q.normalized()
		if !ok {
			return 0, fmt.Errorf("%w: an oblique blend normal has no enclosure", ErrUnsupported)
		}
		point := c.sub(n.mul(obliqueIVOf(radius)))
		worst = max(worst, point.error(held[4+j]))
	}
	return worst, nil
}

// proveObliqueBossFillet combines a cellwise boundary enclosure with a
// conservative occupied-volume enclosure. The volume collar is intentionally
// larger than the actual fillet: it contains *both* source and held changes
// above the original plate; outside it only the cylinder's radial chord
// annulus and the two small cap slabs can differ.
func proveObliqueBossFillet(ctx context.Context, rec *obliqueBossRecord,
	radius float64, mesh *obliqueBossMesh) (float64, float64, float64, error) {
	samples := obliqueBossSamples(rec, radius)
	stride := 5 + obliqueBossBlendCells - 1
	if len(mesh.verts) != len(samples)*stride+2 {
		return 0, 0, 0, fmt.Errorf("%w: the oblique mesh and angular plan disagree", ErrBooleanFailed)
	}
	sampleError := 0.0
	for i, sample := range samples {
		if i%16 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, 0, err
			}
		}
		e, err := obliqueSampleError(rec, radius, sample.u,
			mesh.verts[i*stride:(i+1)*stride])
		if err != nil {
			return 0, 0, 0, err
		}
		sampleError = max(sampleError, e)
	}
	farCenter := obliqueIVVec(rec.base).add(obliqueIVVec(rec.axis).mul(obliqueIVOf(rec.length)))
	sampleError = max(sampleError, farCenter.error(mesh.verts[len(mesh.verts)-1]))
	if proofbound.IsNonFinite(sampleError) {
		return 0, 0, 0, fmt.Errorf("%w: an oblique sample has no finite position bound", ErrUnsupported)
	}
	// The regular 128-part turn limits every gap; inserted square corners can
	// only split a gap. Check the actual ordered plan with rational differences,
	// including its wrap across the enclosed value of 2π.
	angularGap := 0.0
	for i, sample := range samples {
		if i+1 < len(samples) {
			angularGap = max(angularGap, proofbound.RatFloatUp(new(big.Rat).Sub(
				proofarith.FloatRat(samples[i+1].u), proofarith.FloatRat(sample.u))))
			continue
		}
		wrap := new(big.Rat).Add(
			new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper),
				proofarith.FloatRat(sample.u)), proofarith.FloatRat(samples[0].u))
		angularGap = max(angularGap, proofbound.RatFloatUp(wrap))
	}
	if !(angularGap > 0 && angularGap < 0.05) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique angular plan exceeds its chord proof", ErrUnsupported)
	}
	// Admitted tilt and an exact Gram check keep the held canonical basis
	// within 1.01 of orthonormal. These round constants are coarse rational
	// inequalities, not estimates of floating evaluation error.
	if !obliqueBasisAdmitted(rec) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique basis exceeds its proof limits", ErrUnsupported)
	}
	const (
		basis         = 1.01
		minimumNormal = 0.56 // sqrt((1 - 0.35 - 0.01)/2) > 0.56
		maximumQV     = 1.67 // sqrt(1.01^2 + 1 + 2*0.35) < 1.67
	)
	a := proofbound.DivUpper(proofbound.ProductUpper(
		proofbound.AbsSumUpper(rec.radius, radius), basis), 0.93)
	invM := proofbound.DivUpper(1, minimumNormal)
	invM2 := proofbound.ProductUpper(invM, invM)
	twoBasis := proofbound.ProductUpper(2, basis)
	sixBasisSquared := proofbound.ProductUpper(6, proofbound.ProductUpper(basis, basis))
	duu := proofbound.AbsSumUpper(a, proofbound.ProductUpper(radius,
		proofbound.AbsSumUpper(proofbound.ProductUpper(twoBasis, invM),
			proofbound.ProductUpper(sixBasisSquared, invM2))))
	dvv := proofbound.ProductUpper(radius,
		proofbound.ProductUpper(
			proofbound.ProductUpper(6, proofbound.ProductUpper(maximumQV, maximumQV)), invM2))
	duv := proofbound.ProductUpper(radius,
		proofbound.AbsSumUpper(proofbound.ProductUpper(twoBasis, invM),
			proofbound.ProductUpper(proofbound.ProductUpper(6,
				proofbound.ProductUpper(basis, maximumQV)), invM2)))
	vGap := 1.0 / obliqueBossBlendCells
	uSquared := proofbound.ProductUpper(angularGap, angularGap)
	vSquared := proofbound.ProductUpper(vGap, vGap)
	blendChord := proofbound.AbsSumUpper(
		proofbound.ProductUpper(duu, proofbound.DivUpper(uSquared, 8)),
		proofbound.ProductUpper(dvv, proofbound.DivUpper(vSquared, 8)),
		proofbound.ProductUpper(duv,
			proofbound.DivUpper(proofbound.ProductUpper(angularGap, vGap), 4)))
	circleChord := proofbound.ProductUpper(proofbound.AbsSumUpper(rec.radius, radius),
		proofbound.DivUpper(uSquared, 8))
	// Source circle caps certify the entire affine cylinder by ruling between
	// them. Their tiny source-to-ideal displacement is amplified through the
	// plane crossing and rolling-ball normal, whose admitted denominator is
	// at least 0.56 and whose radius is below R/4.
	sourceGap := proofbound.ProductUpper(16, rec.sourceBound)
	// The mesh's cylinder contact uses the retained source's affine radial
	// vector. The smooth blend normal uses its unit direction. The Gram gate
	// bounds the endpoint separation by 0.02r, including both signs of the
	// near-unit radial length.
	normalizationGap := proofbound.ProductUpper(0.02, radius)
	bound := proofbound.AbsSumUpper(sampleError, sourceGap, normalizationGap,
		blendChord, circleChord)
	if proofbound.IsNonFinite(bound) || !(bound < 0.1) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique fillet exceeds the 0.1 mm source boundary proof: %g", ErrUnsupported, bound)
	}
	// Above the plate, the source and held fillets are both inside this full
	// rectangular collar. Charging its entire volume requires no sign or
	// cancellation assumption about the material added by the fillet.
	outerRadius := proofbound.AbsSumUpper(rec.radius, radius, bound)
	widthX := proofbound.AbsSumUpper(proofbound.ProductUpper(rec.length, math.Abs(rec.axis.X)),
		proofbound.DivUpper(proofbound.ProductUpper(2, outerRadius), 0.93))
	widthY := proofbound.AbsSumUpper(proofbound.ProductUpper(rec.length, math.Abs(rec.axis.Y)),
		proofbound.DivUpper(proofbound.ProductUpper(2, outerRadius), 0.93))
	collarHeight := proofbound.AbsSumUpper(proofbound.ProductUpper(radius, 1.35),
		proofbound.ProductUpper(2, bound))
	collarVolume := proofbound.ProductUpper(proofbound.ProductUpper(widthX, widthY), collarHeight)
	// Outside the collar the held boss is a chorded cylinder. The full
	// radial annulus over the cylinder's length and two cap slabs cover its
	// occupied-volume difference from the source cylinder.
	piUpper := proofbound.RatFloatUp(proofbound.PiUpper)
	fourPi := proofbound.ProductUpper(4, piUpper)
	twoPi := proofbound.ProductUpper(2, piUpper)
	annulusVolume := proofbound.ProductUpper(
		proofbound.ProductUpper(fourPi, proofbound.ProductUpper(rec.radius, bound)),
		proofbound.AbsSumUpper(rec.length, proofbound.ProductUpper(2, bound)))
	capSlabs := proofbound.ProductUpper(proofbound.ProductUpper(twoPi,
		proofbound.ProductUpper(outerRadius, outerRadius)), bound)
	volumeGap := proofbound.AbsSumUpper(collarVolume, annulusVolume, capSlabs)
	// A deliberately broad area allowance bounds each retained source patch
	// and the blend separately; the held mesh's own area is proven by exact
	// facet arithmetic. Their sum bounds the absolute area discrepancy.
	heldArea := facetproof.MeshAreaUpper(mesh.verts, mesh.tris)
	w := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.rect.u1), proofarith.FloatRat(rec.rect.u0)))
	h := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.rect.v1), proofarith.FloatRat(rec.rect.v0)))
	plateHeight := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.top), proofarith.FloatRat(rec.bottom)))
	plateArea := proofbound.AbsSumUpper(proofbound.ProductUpper(2, proofbound.ProductUpper(w, h)),
		proofbound.ProductUpper(proofbound.ProductUpper(2, proofbound.AbsSumUpper(w, h)),
			plateHeight))
	bossArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(twoPi, proofbound.ProductUpper(
			proofbound.AbsSumUpper(rec.radius, bound), proofbound.AbsSumUpper(rec.length, bound))),
		proofbound.ProductUpper(piUpper, proofbound.ProductUpper(outerRadius, outerRadius)))
	blendArea := proofbound.ProductUpper(twoPi,
		proofbound.ProductUpper(proofbound.AbsSumUpper(a,
			proofbound.ProductUpper(proofbound.ProductUpper(2,
				proofbound.ProductUpper(radius, basis)), invM)),
			proofbound.ProductUpper(proofbound.ProductUpper(2,
				proofbound.ProductUpper(radius, maximumQV)), invM)))
	areaSlack := proofbound.AbsSumUpper(heldArea,
		proofbound.ProductUpper(2, proofbound.AbsSumUpper(plateArea, bossArea, blendArea)))
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique fillet has no finite volume or area proof", ErrUnsupported)
	}
	return bound, volumeGap, areaSlack, nil
}
