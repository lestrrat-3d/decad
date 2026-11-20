package meshbool

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func TestNativeAxialRimBindsFiniteCrossing(t *testing.T) {
	side, disk, foreign := new(topo.Face), new(topo.Face), new(topo.Face)
	a, b := new(BoolMesh), new(BoolMesh)
	ma := &topo.Mesh{Source: []*topo.Face{side, foreign}}
	mb := &topo.Mesh{Source: []*topo.Face{disk}}
	r := AxialRim{Side: side, Disk: disk, Xlo: big.NewRat(1, 1), Xhi: big.NewRat(1, 1), Zlo: new(big.Rat), Zhi: new(big.Rat), Ylo: new(big.Rat), Yhi: big.NewRat(1, 1), LevelError: .125}
	ctx := WithAxialRims(t.Context(), a, b, ma, mb, []AxialRim{r}, proofbound.NewStrictReplayBudget())
	if axialRimsFor(ctx, b, a) != nil {
		t.Fatal("reused rim for different operands")
	}
	pair := axialRimsFor(ctx, a, b)
	p := proof.Xpt{X: big.NewInt(1), Y: big.NewInt(0), Z: big.NewInt(0), W: big.NewInt(1)}
	q := p
	q.Y = big.NewInt(2)
	got, err := pair.bound(ctx, 0, 0, p, q, 3)
	if err != nil || got < 1.125 || got > 1.126 {
		t.Fatalf("finite crossing bound %g, err %v", got, err)
	}
	got, err = pair.bound(ctx, 1, 0, p, q, 3)
	if err != nil || got != 3 {
		t.Fatalf("foreign wall bound %g, err %v", got, err)
	}
	stop, cancel := context.WithCancel(ctx)
	cancel()
	if _, err := pair.bound(stop, 0, 0, p, q, 3); !errors.Is(err, context.Canceled) {
		t.Fatalf("canceled proof returned %v", err)
	}
}
