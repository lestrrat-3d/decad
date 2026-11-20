package meshbool

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// AxialRim encloses the unique native crossing of a finite extruded line and
// a concentric prism wall. X and Z enclose its exact horizontal position.
// Y is the intersection of the two true axial intervals.
type AxialRim struct {
	Side, Disk                   *topo.Face
	LevelError                   float64
	Xlo, Xhi, Zlo, Zhi, Ylo, Yhi *big.Rat
}
type axialRimKey struct{}
type axialRimPair struct {
	a, b       *BoolMesh
	srcA, srcB []*topo.Face
	rims       []AxialRim
	meter      *proofbound.ReplayBudget
}

// WithAxialRims binds native rim evidence to these exact prepared operand
// meshes. Only the ordinary union producing their contact can read it.
func WithAxialRims(ctx context.Context, a, b *BoolMesh, ma, mb *topo.Mesh, rims []AxialRim, meter *proofbound.ReplayBudget) context.Context {
	return context.WithValue(ctx, axialRimKey{}, &axialRimPair{a, b, ma.Source, mb.Source, rims, meter})
}
func axialRimsFor(ctx context.Context, a, b *BoolMesh) *axialRimPair {
	w, _ := ctx.Value(axialRimKey{}).(*axialRimPair)
	if w == nil || w.a != a || w.b != b {
		return nil
	}
	return w
}
func (w *axialRimPair) bound(ctx context.Context, i, j int, p, q proof.Xpt, orig float64) (float64, error) {
	for _, r := range w.rims {
		if w.srcA[i] != r.Side || w.srcB[j] != r.Disk {
			continue
		}
		if err := w.meter.ReserveBundle(ctx, 640, 16, 32768); err != nil {
			return 0, err
		}
		worst := 0.
		for _, p := range []proof.Xpt{p, q} {
			if p.W.Sign() <= 0 || p.X.BitLen() > 1024 || p.Y.BitLen() > 1024 || p.Z.BitLen() > 1024 || p.W.BitLen() > 1024 {
				return orig, nil
			}
			var total big.Rat
			for _, v := range []struct {
				v      *big.Int
				lo, hi *big.Rat
				plane  bool
			}{{p.X, r.Xlo, r.Xhi, false}, {p.Y, r.Ylo, r.Yhi, true}, {p.Z, r.Zlo, r.Zhi, false}} {
				x := new(big.Rat).SetFrac(v.v, p.W)
				dl := new(big.Rat).Sub(v.lo, x)
				dh := new(big.Rat).Sub(x, v.hi)
				if !v.plane {
					dl.Abs(dl)
					dh.Abs(dh)
				}
				if dh.Cmp(dl) > 0 {
					dl = dh
				}
				if dl.Sign() > 0 {
					total.Add(&total, dl)
				}
			}
			worst = max(worst, proofbound.RatFloatUp(&total))
		}
		return min(proofbound.AbsSumUpper(worst, r.LevelError), orig), nil
	}
	return orig, nil
}
