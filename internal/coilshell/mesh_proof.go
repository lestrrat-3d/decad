package coilshell

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// MeshInput holds the built shell coordinates and their displacement records.
type MeshInput struct {
	Verts    []r3.Vec
	Tris     [][3]int
	Delta    float64
	MaxRound float64
}

// MeshProofs is docs/helix-design.md §8.1's areaSlack and §8.2's
// volSymDiff over the held set: walls triangles first, two per cell in
// (station, loop, segment) order, then both caps.
//
// areaSlack chains every wall cell from its held triangles to the bilinear
// patch on its held corners (the cell's area gap,
// proofbound.CellTwistAreaProjectedAllow, or proofbound.CellTwistAreaAllow
// where that arm states no bound), to the bilinear patch on its true corners
// (each corner moves at most r, the largest station rounding), to the true
// cell (coil.CellProof.Density, which charges an arc chord's own cell
// against its chord's), the last two integrals of the absolute area-density
// gap at one shared parameter; the plane-coordinate legs are carried through
// the denoted map by its stretch and by twice its defect over the bilinear
// patch's own area. Each cap triangle adds proofbound.PerturbedTriangleAreaAllow
// at δ, and each cap adds every arc chord's circular segment, at most
// Arc·Sag.
//
// volSymDiff composes two homotopies of the whole closed boundary: the held
// set to the triangles on the true corners, every vertex moving at most r
// (proofbound.SweptVolumeAllow over the area the motion visits), and those
// triangles to the true walls under the shifted correspondence, a line's cap
// fixed and an arc chord's cap moving inside its own plane, which sweeps no
// volume (coil.CellProof.Swept per cell, carried through the map by |det L|).
func MeshProofs(ctx context.Context, rec Record, mesh MeshInput, walls int) (float64, float64, error) {
	stride := len(rec.Pts)
	n := rec.N
	dt := new(big.Rat).Quo(rec.Turns, big.NewRat(n, 1))
	defect := proofbound.RatFloatUp(rec.Defect)
	det := proofbound.RatFloatUp(new(big.Rat).Abs(rec.Det))
	r := mesh.MaxRound
	cells := float64(n)
	perSegment := make([]float64, stride)
	ends := make([][2]int, stride)
	swept := 0.0
	pos := 0
	for _, idx := range rec.LoopIdx {
		m := len(idx)
		for k := range m {
			v, w := idx[k], idx[(k+1)%m]
			ends[pos] = [2]int{v, w}
			proof, ok := coil.CellProofUpper(rec.Rho[v], rec.Zeta[v], rec.Rho[w], rec.Zeta[w], rec.Profile.Chord[v], rec.Pitch, dt)
			if !ok {
				return 0, 0, fmt.Errorf(`%w: the coil wall of profile segment %d states no mesh proof`, decaderr.ErrUnsupported, v)
			}
			ruling := proofbound.ProductUpper(rec.Stretch, proof.Ruling)
			helix := proofbound.ProductUpper(rec.Stretch, proof.Helix)
			// The bilinear patch on the held corners against the one on the
			// true corners: each derivative moves by at most 2r, so the
			// density moves by at most 2r·|∂s| + (|∂λ| + 2r)·2r.
			twoR := proofbound.ProductUpper(2, r)
			roundLeg := proofbound.AbsSumUpper(
				proofbound.ProductUpper(twoR, helix),
				proofbound.ProductUpper(proofbound.AbsSumUpper(ruling, twoR), twoR),
			)
			perSegment[pos] = proofbound.AbsSumUpper(
				proofbound.ProductUpper(rec.Stretch, proof.Density),
				proofbound.ProductUpper(proofbound.ProductUpper(2, defect), proofbound.ProductUpper(proof.Ruling, proof.Helix)),
				roundLeg,
			)
			swept = proofbound.AbsSumUpper(swept, proofbound.ProductUpper(proof.Swept, cells))
			pos++
		}
	}

	slack := 0.0
	for c := range walls / 2 {
		if c%stride == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, err
			}
		}
		j, p := c/stride, c%stride
		v, w := ends[p][0], ends[p][1]
		at := func(station, vertex int) r3.Vec { return mesh.Verts[station*stride+vertex] }
		twist := proofbound.CellTwistAreaProjectedAllow(at(j, v), at(j+1, v), at(j, w), at(j+1, w))
		if proofbound.IsNonFinite(twist) {
			twist = proofbound.CellTwistAreaAllow(at(j, v), at(j+1, v), at(j, w), at(j+1, w))
		}
		slack = proofbound.AbsSumUpper(slack, twist, perSegment[p])
	}
	for _, t := range mesh.Tris[walls:] {
		slack = proofbound.AbsSumUpper(slack, proofbound.PerturbedTriangleAreaAllow(mesh.Verts[t[0]], mesh.Verts[t[1]], mesh.Verts[t[2]], mesh.Delta))
	}
	// An arc chord's cap triangle stops at the chord: the true cap adds or
	// removes the circular segment between chord and arc, of area at most
	// r²·Δφ³/12 ≤ Arc·Sag, once per cap.
	for _, c := range rec.Profile.Chord {
		if c.Sag == nil {
			continue
		}
		piece := proofbound.RatFloatUp(new(big.Rat).Mul(c.Arc, c.Sag))
		slack = proofbound.AbsSumUpper(slack, proofbound.ProductUpper(2, proofbound.ProductUpper(rec.Stretch, piece)))
	}

	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.Verts, mesh.Tris, r)
	if err != nil {
		return 0, 0, err
	}
	volSymDiff := proofbound.AbsSumUpper(
		proofbound.SweptVolumeAllow(r, area),
		proofbound.ProductUpper(det, swept),
	)
	if proofbound.IsNonFinite(slack) || proofbound.IsNonFinite(volSymDiff) {
		return 0, 0, fmt.Errorf(`%w: the coil states no finite proof for a required mesh bound`, decaderr.ErrUnsupported)
	}
	return slack, volSymDiff, nil
}
