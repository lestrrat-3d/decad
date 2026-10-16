package obliqueboss

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// basisAdmitted reports whether the record's axis tilt and held basis lie
// inside the limits the fillet proof's constants assume.
func basisAdmitted(rec *facetedrecord.ObliqueBossRecord) bool {
	if rec.Axis.Z <= 0.94 {
		return false
	}
	tiltSquared := ObliqueRatDot(r3.Vec{X: rec.Axis.X, Y: rec.Axis.Y},
		r3.Vec{X: rec.Axis.X, Y: rec.Axis.Y})
	tiltLimit := proofarith.FloatRat(0.349)
	if tiltSquared.Cmp(new(big.Rat).Mul(tiltLimit, tiltLimit)) >= 0 {
		return false
	}
	lo, hi := proofarith.FloatRat(0.999), proofarith.FloatRat(1.009)
	lo.Mul(lo, lo)
	hi.Mul(hi, hi)
	for _, basis := range []r3.Vec{rec.Axis, rec.Ex, rec.Ey} {
		square := ObliqueRatDot(basis, basis)
		if square.Cmp(lo) <= 0 || square.Cmp(hi) >= 0 {
			return false
		}
	}
	dotLimit := proofarith.FloatRat(0.0009)
	for _, pair := range [][2]r3.Vec{{rec.Axis, rec.Ex}, {rec.Axis, rec.Ey}, {rec.Ex, rec.Ey}} {
		if new(big.Rat).Abs(ObliqueRatDot(pair[0], pair[1])).Cmp(dotLimit) >= 0 {
			return false
		}
	}
	return true
}

// DiameterBound bounds the diameter of the plate and boss together from the
// rectangle's sides, the plate height, and the boss length and radius, each
// widened by twice boundaryBound.
func DiameterBound(rec *facetedrecord.ObliqueBossRecord, boundaryBound float64) float64 {
	w := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.U1), proofarith.FloatRat(rec.Rect.U0)))
	h := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.V1), proofarith.FloatRat(rec.Rect.V0)))
	plateHeight := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Top), proofarith.FloatRat(rec.Bottom)))
	w = proofbound.AbsSumUpper(w, proofbound.ProductUpper(2, boundaryBound))
	h = proofbound.AbsSumUpper(h, proofbound.ProductUpper(2, boundaryBound))
	z := proofbound.AbsSumUpper(plateHeight, rec.Length, rec.Radius,
		proofbound.ProductUpper(2, boundaryBound))
	return proofbound.Radius3D(max(w, h, z))
}

// patchPointError encloses the exact value of every sampled ideal patch
// point. Trigonometry and normal lengths use rational outward intervals;
// no assumed libm ulp count enters the published boundary bound.
func patchPointError(rec *facetedrecord.ObliqueBossRecord, radius float64,
	u float64, held []r3.Vec) (float64, error) {
	if len(held) != 5+blendCells-1 {
		return 0, fmt.Errorf("%w: an oblique sample has the wrong vertex count", decaderr.ErrBooleanFailed)
	}
	sn, cs, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(u))
	if !ok {
		return 0, fmt.Errorf("%w: an oblique sample has no trigonometric enclosure", decaderr.ErrUnsupported)
	}
	d := ObliqueIVVec(rec.Ex).Mul(cs).Add(ObliqueIVVec(rec.Ey).Mul(sn))
	rPlus := proofbound.IntervalAdd(ObliqueIVOf(rec.Radius), ObliqueIVOf(radius))
	topPlus := proofbound.IntervalAdd(ObliqueIVOf(rec.Top), ObliqueIVOf(radius))
	tNumerator := proofbound.IntervalSub(
		proofbound.IntervalSub(topPlus, ObliqueIVOf(rec.Base.Z)),
		proofbound.IntervalMul(rPlus, d.Z))
	t, ok := proofbound.IntervalQuo(tNumerator, ObliqueIVOf(rec.Axis.Z))
	if !ok {
		return 0, fmt.Errorf("%w: an oblique sample has no axial enclosure", decaderr.ErrUnsupported)
	}
	c := ObliqueIVVec(rec.Base).Add(d.Mul(rPlus)).Add(ObliqueIVVec(rec.Axis).Mul(t))
	vertical := ObliqueZeroPoint()
	vertical.Z = ObliqueIVOf(radius)
	p := c.Sub(vertical)
	s := c.Sub(d.Mul(ObliqueIVOf(radius)))
	far := ObliqueIVVec(rec.Base).Add(d.Mul(ObliqueIVOf(rec.Radius))).
		Add(ObliqueIVVec(rec.Axis).Mul(ObliqueIVOf(rec.Length)))
	worst := max(p.Error(held[2]), s.Error(held[3]), far.Error(held[4]))
	for j := 1; j < blendCells; j++ {
		v := big.NewRat(int64(j), blendCells)
		q := ObliqueZeroPoint()
		q.Z = proofbound.PointInterval(new(big.Rat).Sub(big.NewRat(1, 1), v))
		q = q.Add(d.Mul(proofbound.PointInterval(v)))
		n, ok := q.Normalized()
		if !ok {
			return 0, fmt.Errorf("%w: an oblique blend normal has no enclosure", decaderr.ErrUnsupported)
		}
		point := c.Sub(n.Mul(ObliqueIVOf(radius)))
		worst = max(worst, point.Error(held[4+j]))
	}
	return worst, nil
}

// ProveFillet combines a cellwise boundary enclosure with a
// conservative occupied-volume enclosure. The volume collar is intentionally
// larger than the actual fillet: it contains *both* source and held changes
// above the original plate; outside it only the cylinder's radial chord
// annulus and the two small cap slabs can differ.
func ProveFillet(ctx context.Context, rec *facetedrecord.ObliqueBossRecord,
	radius float64, mesh *Mesh) (float64, float64, float64, error) {
	samples := samplePlan(rec, radius)
	stride := 5 + blendCells - 1
	if len(mesh.Verts) != len(samples)*stride+2 {
		return 0, 0, 0, fmt.Errorf("%w: the oblique mesh and angular plan disagree", decaderr.ErrBooleanFailed)
	}
	sampleError := 0.0
	for i, sample := range samples {
		if i%16 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, 0, err
			}
		}
		e, err := patchPointError(rec, radius, sample.u,
			mesh.Verts[i*stride:(i+1)*stride])
		if err != nil {
			return 0, 0, 0, err
		}
		sampleError = max(sampleError, e)
	}
	farCenter := ObliqueIVVec(rec.Base).Add(ObliqueIVVec(rec.Axis).Mul(ObliqueIVOf(rec.Length)))
	sampleError = max(sampleError, farCenter.Error(mesh.Verts[len(mesh.Verts)-1]))
	if proofbound.IsNonFinite(sampleError) {
		return 0, 0, 0, fmt.Errorf("%w: an oblique sample has no finite position bound", decaderr.ErrUnsupported)
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
		return 0, 0, 0, fmt.Errorf("%w: the oblique angular plan exceeds its chord proof", decaderr.ErrUnsupported)
	}
	// Admitted tilt and an exact Gram check keep the held canonical basis
	// within 1.01 of orthonormal. These round constants are coarse rational
	// inequalities, not estimates of floating evaluation error.
	if !basisAdmitted(rec) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique basis exceeds its proof limits", decaderr.ErrUnsupported)
	}
	const (
		basis         = 1.01
		minimumNormal = 0.56 // sqrt((1 - 0.35 - 0.01)/2) > 0.56
		maximumQV     = 1.67 // sqrt(1.01^2 + 1 + 2*0.35) < 1.67
	)
	a := proofbound.DivUpper(proofbound.ProductUpper(
		proofbound.AbsSumUpper(rec.Radius, radius), basis), 0.93)
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
	vGap := 1.0 / blendCells
	uSquared := proofbound.ProductUpper(angularGap, angularGap)
	vSquared := proofbound.ProductUpper(vGap, vGap)
	blendChord := proofbound.AbsSumUpper(
		proofbound.ProductUpper(duu, proofbound.DivUpper(uSquared, 8)),
		proofbound.ProductUpper(dvv, proofbound.DivUpper(vSquared, 8)),
		proofbound.ProductUpper(duv,
			proofbound.DivUpper(proofbound.ProductUpper(angularGap, vGap), 4)))
	circleChord := proofbound.ProductUpper(proofbound.AbsSumUpper(rec.Radius, radius),
		proofbound.DivUpper(uSquared, 8))
	// Source circle caps certify the entire affine cylinder by ruling between
	// them. Their tiny source-to-ideal displacement is amplified through the
	// plane crossing and rolling-ball normal, whose admitted denominator is
	// at least 0.56 and whose radius is below R/4.
	sourceGap := proofbound.ProductUpper(16, rec.SourceBound)
	// The mesh's cylinder contact uses the retained source's affine radial
	// vector. The smooth blend normal uses its unit direction. The Gram gate
	// bounds the endpoint separation by 0.02r, including both signs of the
	// near-unit radial length.
	normalizationGap := proofbound.ProductUpper(0.02, radius)
	bound := proofbound.AbsSumUpper(sampleError, sourceGap, normalizationGap,
		blendChord, circleChord)
	if proofbound.IsNonFinite(bound) || !(bound < 0.1) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique fillet exceeds the 0.1 mm source boundary proof: %g", decaderr.ErrUnsupported, bound)
	}
	// Above the plate, the source and held fillets are both inside this full
	// rectangular collar. Charging its entire volume requires no sign or
	// cancellation assumption about the material added by the fillet.
	outerRadius := proofbound.AbsSumUpper(rec.Radius, radius, bound)
	widthX := proofbound.AbsSumUpper(proofbound.ProductUpper(rec.Length, math.Abs(rec.Axis.X)),
		proofbound.DivUpper(proofbound.ProductUpper(2, outerRadius), 0.93))
	widthY := proofbound.AbsSumUpper(proofbound.ProductUpper(rec.Length, math.Abs(rec.Axis.Y)),
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
		proofbound.ProductUpper(fourPi, proofbound.ProductUpper(rec.Radius, bound)),
		proofbound.AbsSumUpper(rec.Length, proofbound.ProductUpper(2, bound)))
	capSlabs := proofbound.ProductUpper(proofbound.ProductUpper(twoPi,
		proofbound.ProductUpper(outerRadius, outerRadius)), bound)
	volumeGap := proofbound.AbsSumUpper(collarVolume, annulusVolume, capSlabs)
	// A deliberately broad area allowance bounds each retained source patch
	// and the blend separately; the held mesh's own area is proven by exact
	// facet arithmetic. Their sum bounds the absolute area discrepancy.
	heldArea := facetproof.MeshAreaUpper(mesh.Verts, mesh.Tris)
	w := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.U1), proofarith.FloatRat(rec.Rect.U0)))
	h := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.V1), proofarith.FloatRat(rec.Rect.V0)))
	plateHeight := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Top), proofarith.FloatRat(rec.Bottom)))
	plateArea := proofbound.AbsSumUpper(proofbound.ProductUpper(2, proofbound.ProductUpper(w, h)),
		proofbound.ProductUpper(proofbound.ProductUpper(2, proofbound.AbsSumUpper(w, h)),
			plateHeight))
	bossArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(twoPi, proofbound.ProductUpper(
			proofbound.AbsSumUpper(rec.Radius, bound), proofbound.AbsSumUpper(rec.Length, bound))),
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
		return 0, 0, 0, fmt.Errorf("%w: the oblique fillet has no finite volume or area proof", decaderr.ErrUnsupported)
	}
	return bound, volumeGap, areaSlack, nil
}
