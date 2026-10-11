package loftmesh

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestParallelLoftAuditPreservesOrderedResults(t *testing.T) {
	const sectors = 455 // More than 100,000 overlapping-box candidate pairs.
	verts := []r3.Vec{r3.NewVec(0, 0, 0)}
	tris := make([][3]int, 0, sectors+1)
	step := 2 * math.Pi / sectors
	for i := range sectors {
		a := step * float64(i)
		b := a + step/4
		start := len(verts)
		verts = append(verts, r3.NewVec(math.Cos(a), math.Sin(a), 0),
			r3.NewVec(math.Cos(b), math.Sin(b), 0))
		tris = append(tris, [3]int{0, start, start + 1})
	}
	shortcuts := loftGenericShortcuts
	check := func(name string, tris [][3]int, failing bool) {
		t.Helper()
		var want LoftAuditWork
		var wantErr error
		for _, parallel := range []bool{false, true} {
			work, err := loftCrossingAuditLimited(proofbound.NewWorkBudget(context.Background()),
				verts, tris, nil, shortcuts, 8_000_000, 1_000, parallel)
			if !parallel {
				want, wantErr = work, err
				if work.Candidates < 100_000 {
					t.Fatalf("%s: only %d candidate pairs", name, work.Candidates)
				}
				if (err != nil) != failing {
					t.Fatalf("%s: serial result = %v, want failure %t", name, err, failing)
				}
				continue
			}
			if work != want {
				t.Errorf("%s: parallel work = %+v, serial work = %+v", name, work, want)
			}
			if err == nil && wantErr != nil || err != nil && wantErr == nil ||
				err != nil && wantErr != nil && err.Error() != wantErr.Error() {
				t.Errorf("%s: parallel error = %v, serial error = %v", name, err, wantErr)
			}
		}
	}
	check("clean", tris, false)
	start := len(verts)
	verts = append(verts, verts[1], verts[2])
	tris = append(tris, [3]int{0, start, start + 1})
	check("first refusal", tris, true)
}
