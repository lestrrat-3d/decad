package meshbool

import (
	"cmp"
	"errors"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func TestBrakeConformSpatialIndexKeepsExactHits(t *testing.T) {
	for _, offset := range []float64{0, 1e12} {
		point := func(x, y, z float64) proof.Xpt {
			return proof.XptOf(r3.Vec{X: offset + x, Y: offset + y, Z: offset + z})
		}
		verts := []proof.Xpt{point(0, 0, 0), point(10, 10, 10), point(5, 5, 5)}
		for _, x := range []float64{9, 1, 5, 7, 3} {
			verts = append(verts, point(x, x, x))
		}
		for i := range 97 {
			x := float64(i) / 10
			verts = append(verts, point(x, x+0.1, x+0.2))
		}
		meter := proofbound.NewBrakeReplayBudget()
		budget := meter.WorkBudget(t.Context())
		scan, err := brakeNewConformScan(t.Context(), budget, verts)
		if err != nil {
			t.Fatal(err)
		}
		tri := [3]int{0, 1, 2}
		for _, edge := range [][2]int{{0, 1}, {1, 0}, {0, 2}, {0, 0}} {
			var want []int
			for i := range verts {
				if i == tri[0] || i == tri[1] || i == tri[2] {
					continue
				}
				if OnSegmentInterior3(verts[edge[0]], verts[edge[1]], verts[i]) {
					want = append(want, i)
				}
			}
			slices.SortFunc(want, func(a, b int) int {
				if order := cmp.Compare(scan.Approx[a].X, scan.Approx[b].X); order != 0 {
					return order
				}
				return cmp.Compare(a, b)
			})
			got, err := scan.EdgeInteriorHits(t.Context(), budget, edge[0], edge[1], tri)
			if err != nil || !slices.Equal(want, got) {
				t.Fatalf("offset=%g edge=%v want=%v got=%v err=%v", offset, edge, want, got, err)
			}
			pa, pb := scan.Approx[edge[0]], scan.Approx[edge[1]]
			tau := math.Nextafter(math.Sqrt(scan.Tau2), math.Inf(1))
			lo := r3.Vec{X: math.Nextafter(min(pa.X, pb.X)-tau, math.Inf(-1)), Y: math.Nextafter(min(pa.Y, pb.Y)-tau, math.Inf(-1)), Z: math.Nextafter(min(pa.Z, pb.Z)-tau, math.Inf(-1))}
			hi := r3.Vec{X: math.Nextafter(max(pa.X, pb.X)+tau, math.Inf(1)), Y: math.Nextafter(max(pa.Y, pb.Y)+tau, math.Inf(1)), Z: math.Nextafter(max(pa.Z, pb.Z)+tau, math.Inf(1))}
			for i, p := range scan.Approx {
				candidate := p.X >= lo.X && p.X <= hi.X && p.Y >= lo.Y && p.Y <= hi.Y && p.Z >= lo.Z && p.Z <= hi.Z
				stored := scan.pointBits[i/64]&(uint64(1)<<uint(i%64)) != 0
				if stored != candidate {
					t.Fatalf("offset=%g edge=%v candidate %d mismatch", offset, edge, i)
				}
			}
		}
	}
}

func TestBrakeConformSpatialIndexResourceLimits(t *testing.T) {
	ctx := t.Context()
	verts := []proof.Xpt{proof.XptOf(r3.Vec{}), proof.XptOf(r3.Vec{X: 10, Y: 10, Z: 10}), proof.XptOf(r3.Vec{Y: 10}), proof.XptOf(r3.Vec{X: 5, Y: 5, Z: 5})}
	for i := range 97 {
		verts = append(verts, proof.XptOf(r3.Vec{X: float64(i) / 10, Y: 2, Z: 3}))
	}
	before := slices.Clone(verts)
	values := make([][4]string, len(verts))
	for i, p := range verts {
		values[i] = [4]string{p.X.String(), p.Y.String(), p.Z.String(), p.W.String()}
	}
	run := func(meter *proofbound.ReplayBudget) ([]int, error) {
		budget := meter.WorkBudget(ctx)
		scan, err := brakeNewConformScan(ctx, budget, verts)
		if err != nil {
			return nil, err
		}
		return scan.EdgeInteriorHits(ctx, budget, 0, 1, [3]int{0, 1, 2})
	}
	measured := proofbound.NewBrakeReplayBudget()
	want, err := run(measured)
	if err != nil || !slices.Equal(want, []int{3}) {
		t.Fatalf("measuring hits=%v err=%v", want, err)
	}
	w, r := measured.Usage()
	z := measured.Bytes()
	for _, resource := range []string{brakeTestExact, brakeTestWork, brakeTestRecords, brakeTestBytes} {
		t.Run(resource, func(t *testing.T) {
			meter := proofbound.NewBrakeReplayBudget()
			work, records, bytes := proofbound.BrakeReplayWork-w, proofbound.BrakeReplayRecords-r, proofbound.BrakeReplayBytes-z
			switch resource {
			case brakeTestWork:
				work++
			case brakeTestRecords:
				records++
			case brakeTestBytes:
				bytes++
			}
			if err := meter.SpendWork(ctx, work); err != nil {
				t.Fatal(err)
			}
			if err := meter.ReserveRecords(ctx, records); err != nil {
				t.Fatal(err)
			}
			if err := meter.ReserveBytes(ctx, bytes); err != nil {
				t.Fatal(err)
			}
			hits, err := run(meter)
			if resource == brakeTestExact {
				if err != nil || !slices.Equal(hits, want) {
					t.Fatalf("exact hits=%v err=%v", hits, err)
				}
			} else {
				var detail *proofbound.WorkLimitError
				expected := map[string]string{brakeTestWork: "multiway_work", brakeTestRecords: "multiway_records", brakeTestBytes: "replay_bytes"}[resource]
				if !errors.As(err, &detail) || detail.Resource != expected || hits != nil {
					t.Fatalf("one-short %s: %v", resource, err)
				}
			}
			if !slices.Equal(before, verts) {
				t.Fatal("index query changed exact vertex pointers")
			}
			for i, p := range verts {
				if values[i] != [4]string{p.X.String(), p.Y.String(), p.Z.String(), p.W.String()} {
					t.Fatalf("index query changed exact vertex %d", i)
				}
			}
		})
	}
}
