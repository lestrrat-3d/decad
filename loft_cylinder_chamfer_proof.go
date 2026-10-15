package decad

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// The ideal side stations are exact rational points on the old wall. Their
// rounded output images are charged separately after this ideal audit.
func loftCylinderChamferIdealVertices(verts []r3.Vec, side loftCylinderChamferSide) []proofarith.Xpt {
	xverts := make([]proofarith.Xpt, len(verts))
	for i, v := range verts {
		xverts[i] = proofarith.XptOf(v)
	}
	for cap, ring := range side.rings {
		for i, vertex := range ring {
			xverts[vertex] = side.exact[cap][i]
		}
	}
	return xverts
}

// A mid-height point well inside the exact blank, and outside the bore when
// present, fixes the occupied side. The ideal middle side is an exact subset
// of the old wall's two-triangle strips, so it has the same classification
// there; the cap bands cannot reach this level.
func loftCylinderChamferSolidWitness(ctx context.Context, fp facetedPayload,
	source loftCylinderChamferSource, setback float64) error {
	mid := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(source.z1))
	mid.Quo(mid, big.NewRat(2, 1))
	start := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(setback))
	end := new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(setback))
	if mid.Cmp(start) <= 0 || mid.Cmp(end) >= 0 {
		return fmt.Errorf("%w: cap bands leave no common interior slice", ErrUnsupported)
	}
	r := 0.0
	if source.boreRadius > 0 {
		r = proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Add(
			proofarith.FloatRat(source.boreRadius), proofarith.FloatRat(source.blankRadius)),
			big.NewRat(2, 1)))
	}
	if proofbound.AbsSumUpper(r, source.innerInset) >= source.blankRadius ||
		source.boreRadius > 0 && r <= proofbound.AbsSumUpper(source.boreRadius, fp.MeshBound) {
		return fmt.Errorf("%w: cap chamfer lacks an interior blank witness", ErrUnsupported)
	}
	witness := proofarith.Xlerp(
		proofarith.XptOf(r3.Vec{X: r, Z: source.z0}),
		proofarith.XptOf(r3.Vec{X: r, Z: source.z1}),
		big.NewInt(1), big.NewInt(2))
	indices := make([]int, len(fp.Tris))
	for i := range indices {
		indices[i] = i
	}
	inside, on, err := meshbool.MeshParityPreparedContext(ctx, witness,
		meshbool.NewParityMesh(fp.Verts, fp.Tris), indices)
	if err != nil {
		return err
	}
	if !inside || on {
		return fmt.Errorf("%w: cap chamfer shell excludes its blank witness", ErrUnsupported)
	}
	return nil
}

type loftCylinderChamferToothFace struct {
	body  *Body
	loft  loftPayload
	root  int
	cell  int
	bound float64
	scale *big.Rat
}

// Union copied these precise Loft face triangles, and Cut retained their
// groups. Match by authenticated origin instead of interpreting a role string
// on a faceted result that may coalesce several public edges.
func loftCylinderChamferToothFaces(source loftCylinderChamferSource) (map[provenance.Ref][]loftCylinderChamferToothFace, error) {
	if source.blank == nil || len(source.teeth) == 0 ||
		len(source.originalTeeth) != len(source.teeth) || source.blankRadius <= 0 {
		return nil, fmt.Errorf("%w: loft/cylinder chamfer lost its source operands", ErrUnsupported)
	}
	faces := make(map[provenance.Ref][]loftCylinderChamferToothFace)
	for toothIndex, tooth := range source.teeth {
		if tooth == nil || source.originalTeeth[toothIndex] == nil {
			return nil, fmt.Errorf("%w: loft/cylinder tooth source is absent", ErrUnsupported)
		}
		lp, ok := tooth.n.Payload.(loftPayload)
		original, originalOK := source.originalTeeth[toothIndex].n.Payload.(loftPayload)
		if !ok || !lp.AuthenticatedReconstruction || lp.SourceIdentity == nil ||
			!originalOK || original.SourceIdentity != lp.SourceIdentity ||
			lp.Proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.Proof.FacetDeparture) ||
			len(lp.Segment) != 1 {
			return nil, fmt.Errorf("%w: loft/cylinder tooth lost its certified surface", ErrUnsupported)
		}
		root := -1
		for i, segment := range lp.Segment[0] {
			if segment == 5 {
				root = i
				break
			}
		}
		toothFaces := tooth.Faces()
		if root < 3 || len(toothFaces) != lp.Walls+2 || lp.Walls < 2*root {
			return nil, fmt.Errorf("%w: loft/cylinder tooth has no paired exterior faces", ErrUnsupported)
		}
		for j := range root {
			for side := range 2 {
				f := toothFaces[2+2*j+side]
				if len(f.n.Origins) == 0 {
					return nil, fmt.Errorf("%w: loft/cylinder tooth face lost its origin", ErrUnsupported)
				}
				origin := f.n.Origins[0]
				faces[origin] = append(faces[origin], loftCylinderChamferToothFace{
					body: tooth, loft: lp, root: root, cell: j, bound: lp.Proof.FacetDeparture,
					scale: proofarith.FloatRat(original.Xform.Basis().EZ.Z),
				})
			}
		}
	}
	return faces, nil
}
