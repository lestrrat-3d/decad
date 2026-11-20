package rimextension

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestAlignedTerminalDiskCrossing(t *testing.T) {
	n := windowNative{x0: scalar(-3), z0: scalar(4), m: scalar(.75), last: scalar(-3), cx: scalar(0), cz: scalar(0), r: scalar(5), out: 1}
	check := func() (shoulderDiskRange, bool) {
		v, ok, err := shoulderDiskRangeAt(t.Context(), n, scalar(-6), scalar(.4), true, proofbound.NewStrictReplayBudget())
		if err != nil {
			t.Fatal(err)
		}
		return v, ok
	}
	v, ok := check()
	if !ok || v.crossing.Cmp(scalar(-3.000001)) <= 0 || v.crossing.Cmp(n.last) > 0 {
		t.Fatal("missing finite tangent crossing")
	}
	n.z0 = scalar(4.000001)
	if _, ok := check(); ok {
		t.Fatal("accepted terminal beyond the pad")
	}
	n.z0 = scalar(3.999999)
	if _, ok := check(); !ok {
		t.Fatal("rejected terminal inside the pad")
	}
}
