package revolveloftfaces

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// EmbeddingSection carries only the cap mesh and its source map inputs.
type EmbeddingSection struct {
	Mesh        facetedrecord.Payload
	Limit       *facetedrecord.PointPlaneLimit
	Constructed *loftface.Map
}

func lfZero() *big.Rat { return new(big.Rat) }
func lfOne() *big.Rat  { return big.NewRat(1, 1) }

func capSimilarity(section EmbeddingSection) (loftface.Affine, error) {
	out := loftface.Identity()
	if section.Mesh.PointSimilarity != nil {
		for _, step := range section.Mesh.PointSimilarity.Steps {
			if step.Factor == 0 {
				if !step.Motion.IsValid() {
					return loftface.Affine{}, fmt.Errorf("%w: loft cap has an invalid rigid step", decaderr.ErrUnsupported)
				}
				out = loftface.Compose(out, loftface.Motion(step.Motion))
			} else {
				if !proofbound.FiniteVec(step.Anchor) || !proofbound.FiniteVec(r3.Vec{X: step.Factor}) ||
					step.Factor <= 0 {
					return loftface.Affine{}, fmt.Errorf("%w: loft cap has an invalid scale step", decaderr.ErrUnsupported)
				}
				out = loftface.Compose(out, loftface.Scale(step.Anchor, step.Factor))
			}
		}
	} else if section.Mesh.Xform != r3.Identity() {
		if !section.Mesh.Xform.IsValid() {
			return loftface.Affine{}, fmt.Errorf("%w: loft cap has an invalid placement", decaderr.ErrUnsupported)
		}
		out = loftface.Compose(out, loftface.Motion(section.Mesh.Xform))
	}
	return out, nil
}

func capMap(section EmbeddingSection, source *facetedrecord.PointSectionRecord) (loftface.Map, error) {
	if section.Constructed != nil {
		return *section.Constructed, nil
	}
	affine, err := capSimilarity(section)
	if err != nil {
		return loftface.Map{}, err
	}
	apex := loftface.Vec(source.Apex)
	origin := loftface.Vec(source.Plane.Origin)
	uAxis := loftface.Vec(source.Plane.U)
	vAxis := loftface.Vec(source.Plane.V)
	var n loftface.RatVec
	var d *big.Rat
	if section.Limit == nil {
		n = loftface.RatVec{lfZero(), lfZero(), lfZero()}
		d = lfOne()
	} else {
		n = loftface.RatVec{section.Limit.Nx, section.Limit.Ny, lfZero()}
		d = section.Limit.D
	}
	g0, gu, gv := loftface.VDot(n, loftface.VSub(origin, apex)), loftface.VDot(n, uAxis), loftface.VDot(n, vAxis)
	if section.Limit == nil {
		g0 = lfOne()
	}
	q0 := loftface.VScale(loftface.MatVec(affine.A, loftface.VSub(origin, apex)), d)
	qu := loftface.VScale(loftface.MatVec(affine.A, uAxis), d)
	qv := loftface.VScale(loftface.MatVec(affine.A, vAxis), d)
	stretch := loftface.AffineStretch(affine.A)
	if section.Limit != nil {
		stretch = proofbound.ProductUpper(stretch, section.Limit.MapStretch)
	}
	if proofbound.IsNonFinite(stretch) {
		return loftface.Map{}, fmt.Errorf("%w: loft cap source stretch is not finite", decaderr.ErrUnsupported)
	}
	return loftface.Map{Constant: loftface.VAdd(loftface.MatVec(affine.A, apex), affine.B),
		Q0: q0, QU: qu, QV: qv, G0: g0, GU: gu, GV: gv, Stretch: stretch}, nil
}

type Embedding struct {
	Maps        []loftface.Map
	JacobianSum float64
	MaximumNorm float64
	Sign        int
}

func ProveEmbedding(ctx context.Context, sections []EmbeddingSection,
	source *facetedrecord.PointSectionRecord) (Embedding, error) {
	var proof Embedding
	box, err := LfSourceBox(source)
	if err != nil {
		return proof, err
	}
	proof.Maps = make([]loftface.Map, len(sections))
	for i, section := range sections {
		if err := ctx.Err(); err != nil {
			return proof, err
		}
		proof.Maps[i], err = capMap(section, source)
		if err != nil {
			return proof, fmt.Errorf("loft section %d: %w", i, err)
		}
		if err := loftface.PositiveDenominator(proof.Maps[i], box); err != nil {
			return proof, fmt.Errorf("loft section %d: %w", i, err)
		}
	}
	if err := loftface.PlaneOrder(proof.Maps, box); err != nil {
		return proof, err
	}
	for band := 0; band+1 < len(proof.Maps); band++ {
		if err := ctx.Err(); err != nil {
			return proof, err
		}
		rho, jac, sign, gateErr := loftface.BandGate(ctx, proof.Maps[band], proof.Maps[band+1], box)
		if gateErr != nil {
			return proof, fmt.Errorf("loft band %d: %w", band, gateErr)
		}
		if proof.Sign != 0 && proof.Sign != sign {
			return proof, fmt.Errorf("%w: loft bands reverse their source orientation", decaderr.ErrUnsupported)
		}
		proof.Sign = sign
		proof.MaximumNorm = max(proof.MaximumNorm, rho)
		proof.JacobianSum = proofbound.AbsSumUpper(proof.JacobianSum, jac)
	}
	return proof, nil
}

func Orient(mesh *facetedrecord.Payload, base facetedrecord.Payload,
	source *facetedrecord.PointSectionRecord, jacobianSign int) error {
	n := len(source.SourceStation)
	if n < 3 || len(base.Tris) <= n || jacobianSign == 0 {
		return fmt.Errorf("%w: loft source cap has no orientation", decaderr.ErrUnsupported)
	}
	polygonSign := 0
	for _, tri := range base.Tris[n:] {
		if tri[0] <= 0 || tri[1] <= 0 || tri[2] <= 0 ||
			tri[0] > n || tri[1] > n || tri[2] > n {
			return fmt.Errorf("%w: loft source cap has invalid station indices", decaderr.ErrUnsupported)
		}
		a, b, c := source.SourceStation[tri[0]-1], source.SourceStation[tri[1]-1],
			source.SourceStation[tri[2]-1]
		cross := lfSub(lfMul(lfSub(lfF(b.U), lfF(a.U)), lfSub(lfF(c.V), lfF(a.V))),
			lfMul(lfSub(lfF(b.V), lfF(a.V)), lfSub(lfF(c.U), lfF(a.U))))
		sign := cross.Sign()
		if sign == 0 || (polygonSign != 0 && sign != polygonSign) {
			return fmt.Errorf("%w: loft source cap triangulation changes orientation", decaderr.ErrUnsupported)
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
