package decad

import (
	"errors"
	"testing"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

func TestReplayHandleAllocationSizes(t *testing.T) {
	kinds := []struct {
		kind string
		size uint64
	}{
		{"body", uint64(unsafe.Sizeof(Body{}))}, {"lump", uint64(unsafe.Sizeof(Lump{}))},
		{"shell", uint64(unsafe.Sizeof(FaceShell{}))}, {"face", uint64(unsafe.Sizeof(Face{}))},
		{"loop", uint64(unsafe.Sizeof(Loop{}))}, {"edge", uint64(unsafe.Sizeof(Edge{}))},
		{"vertex", uint64(unsafe.Sizeof(Vertex{}))},
	}
	for _, k := range kinds {
		t.Run(k.kind, func(t *testing.T) {
			size, err := topo.ReplayNodeBytes(handleAlloc{}, k.kind)
			if err != nil || size != k.size {
				t.Fatalf("size=%d want=%d error=%v", size, k.size, err)
			}
			for _, short := range []bool{false, true} {
				meter := proofbound.NewBrakeReplayBudget()
				remaining := 2*size + 128
				if short {
					remaining--
				}
				if err := meter.ReserveBytes(t.Context(), proofbound.BrakeReplayBytes-remaining); err != nil {
					t.Fatal(err)
				}
				err := meter.WorkBudget(t.Context()).ReserveBytesMul(1, size)
				if short && !errors.Is(err, decaderr.ErrUnsupported) {
					t.Fatalf("one-short error: %v", err)
				}
				if !short && err != nil {
					t.Fatalf("exact size error: %v", err)
				}
			}
		})
	}
	if _, err := topo.ReplayNodeBytes(handleAlloc{}, "unknown"); err == nil {
		t.Fatal("unknown kind accepted")
	}
}
