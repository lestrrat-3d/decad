package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/r3"
)

// The LoftFaces cap-map carriers live in internal/loftface.
type loftFaceRatVec = loftface.RatVec
type loftFaceAffine = loftface.Affine
type loftFaceMap = loftface.Map
type loftFaceInterval = loftface.Interval

func lfZero() *big.Rat       { return new(big.Rat) }
func lfOne() *big.Rat        { return big.NewRat(1, 1) }
func lfF(v float64) *big.Rat { return ratOfPointPlane(v) }

// lfAdd, lfSub, lfMul and lfDiv return a fresh Rat and read their operands only.
// Operands must be in lowest terms: the result always equals the exact value,
// but is in lowest terms only for reduced operands (a 3/9 operand gives 3/9
// where big.Rat gives 1/3). Every producer in this package reduces: FloatRat
// through lfF and ratOfPointPlane, big.NewRat literals, lfOne, lfZero, and
// results of these helpers and of raw Quo, Abs and Neg. The guard
// TestNoRatWritesThroughNumDenom (internal/proof) bars in-place Num and Denom
// writes outside internal/proof. lfDiv by zero panics as Rat.Quo does.
func lfAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }
func lfSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }
func lfMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }
func lfDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }
func lfAbs(a *big.Rat) *big.Rat    { return new(big.Rat).Abs(a) }

func lfSimilarity(section loftFaceSection) (loftFaceAffine, error) {
	out := loftface.Identity()
	if section.mesh.PointSimilarity != nil {
		for _, step := range section.mesh.PointSimilarity.Steps {
			if step.Factor == 0 {
				if !step.Motion.IsValid() {
					return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid rigid step", ErrUnsupported)
				}
				out = loftface.Compose(out, loftface.Motion(step.Motion))
			} else {
				if !proofbound.FiniteVec(step.Anchor) || !proofbound.FiniteVec(r3.Vec{X: step.Factor}) ||
					step.Factor <= 0 {
					return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid scale step", ErrUnsupported)
				}
				out = loftface.Compose(out, loftface.Scale(step.Anchor, step.Factor))
			}
		}
	} else if section.mesh.Xform != r3.Identity() {
		if !section.mesh.Xform.IsValid() {
			return loftFaceAffine{}, fmt.Errorf("%w: loft cap has an invalid placement", ErrUnsupported)
		}
		out = loftface.Compose(out, loftface.Motion(section.mesh.Xform))
	}
	return out, nil
}

func lfCapMap(section loftFaceSection, source *pointSectionRecord) (loftFaceMap, error) {
	if section.constructed != nil {
		return *section.constructed, nil
	}
	affine, err := lfSimilarity(section)
	if err != nil {
		return loftFaceMap{}, err
	}
	apex := loftface.Vec(source.Apex)
	origin := loftface.Vec(source.Plane.Origin)
	uAxis := loftface.Vec(source.Plane.U)
	vAxis := loftface.Vec(source.Plane.V)
	var n loftFaceRatVec
	var d *big.Rat
	if section.limit == nil {
		n = loftFaceRatVec{lfZero(), lfZero(), lfZero()}
		d = lfOne()
	} else {
		n = loftFaceRatVec{section.limit.Nx, section.limit.Ny, lfZero()}
		d = section.limit.D
	}
	g0, gu, gv := loftface.VDot(n, loftface.VSub(origin, apex)), loftface.VDot(n, uAxis), loftface.VDot(n, vAxis)
	if section.limit == nil {
		g0 = lfOne()
	}
	q0 := loftface.VScale(loftface.MatVec(affine.A, loftface.VSub(origin, apex)), d)
	qu := loftface.VScale(loftface.MatVec(affine.A, uAxis), d)
	qv := loftface.VScale(loftface.MatVec(affine.A, vAxis), d)
	stretch := loftface.AffineStretch(affine.A)
	if section.limit != nil {
		stretch = proofbound.ProductUpper(stretch, section.limit.MapStretch)
	}
	if proofbound.IsNonFinite(stretch) {
		return loftFaceMap{}, fmt.Errorf("%w: loft cap source stretch is not finite", ErrUnsupported)
	}
	return loftFaceMap{Constant: loftface.VAdd(loftface.MatVec(affine.A, apex), affine.B),
		Q0: q0, QU: qu, QV: qv, G0: g0, GU: gu, GV: gv, Stretch: stretch}, nil
}

type loftFaceEmbedding struct {
	maps        []loftFaceMap
	jacobianSum float64
	maximumNorm float64
	sign        int
}

func lfStationUV(point planePoint) (*big.Rat, *big.Rat) { return lfF(point.U), lfF(point.V) }

func proveLoftFaceEmbedding(ctx context.Context, sections []loftFaceSection,
	source *pointSectionRecord) (loftFaceEmbedding, error) {
	var proof loftFaceEmbedding
	box, err := revolveloftfaces.LfSourceBox(source)
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
		if err := loftface.PositiveDenominator(proof.maps[i], box); err != nil {
			return proof, fmt.Errorf("loft section %d: %w", i, err)
		}
	}
	if err := loftface.PlaneOrder(proof.maps, box); err != nil {
		return proof, err
	}
	for band := 0; band+1 < len(proof.maps); band++ {
		if err := ctx.Err(); err != nil {
			return proof, err
		}
		rho, jac, sign, gateErr := loftface.BandGate(ctx, proof.maps[band], proof.maps[band+1], box)
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
	n := len(source.SourceStation)
	if n < 3 || len(base.Tris) <= n || jacobianSign == 0 {
		return fmt.Errorf("%w: loft source cap has no orientation", ErrUnsupported)
	}
	polygonSign := 0
	for _, tri := range base.Tris[n:] {
		if tri[0] <= 0 || tri[1] <= 0 || tri[2] <= 0 ||
			tri[0] > n || tri[1] > n || tri[2] > n {
			return fmt.Errorf("%w: loft source cap has invalid station indices", ErrUnsupported)
		}
		a, b, c := source.SourceStation[tri[0]-1], source.SourceStation[tri[1]-1],
			source.SourceStation[tri[2]-1]
		cross := lfSub(lfMul(lfSub(lfF(b.U), lfF(a.U)), lfSub(lfF(c.V), lfF(a.V))),
			lfMul(lfSub(lfF(b.V), lfF(a.V)), lfSub(lfF(c.U), lfF(a.U))))
		sign := cross.Sign()
		if sign == 0 || (polygonSign != 0 && sign != polygonSign) {
			return fmt.Errorf("%w: loft source cap triangulation changes orientation", ErrUnsupported)
		}
		polygonSign = sign
	}
	if polygonSign != jacobianSign {
		for i := range mesh.Tris {
			mesh.Tris[i][1], mesh.Tris[i][2] = mesh.Tris[i][2], mesh.Tris[i][1]
		}
		mesh.Groups[len(mesh.Groups)-2].Reversed = false
		mesh.Groups[len(mesh.Groups)-1].Reversed = true
	}
	return nil
}
