package meshbool

import (
	"context"
	"errors"
	"testing"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

const (
	brakeTestExact   = "exact"
	brakeTestRecords = "records"
	brakeTestWork    = "work"
	brakeTestBytes   = "bytes"
)

func TestBrakeCollectorFixedReservation(t *testing.T) {
	for _, short := range []string{brakeTestExact, brakeTestRecords, brakeTestBytes} {
		meter := proofbound.NewBrakeReplayBudget()
		remainingBytes := 2*(uint64(unsafe.Sizeof(PairCutsResult{}))+replayCollectorFixedBytes) + 128
		remainingRecords := uint64(replayCollectorFixedRecords)
		if short == brakeTestBytes {
			remainingBytes--
		}
		if short == brakeTestRecords {
			remainingRecords--
		}
		if err := meter.ReserveBytes(t.Context(), proofbound.BrakeReplayBytes-remainingBytes); err != nil {
			t.Fatal(err)
		}
		if err := meter.ReserveRecords(t.Context(), proofbound.BrakeReplayRecords-remainingRecords); err != nil {
			t.Fatal(err)
		}
		budget, err := NewReplayPairCutsBudget(meter)
		if err != nil {
			t.Fatal(err)
		}
		err = reserveReplayCollectorFixed(t.Context(), budget)
		if short != brakeTestExact && !errors.Is(err, decaderr.ErrUnsupported) {
			t.Fatalf("one-short %s: %v", short, err)
		}
		if short == brakeTestExact && err != nil {
			t.Fatalf("exact: %v", err)
		}
	}
}

// TestBrakeCollectorFixedEnvelope checks the compiler-observed capture shapes.
func TestBrakeCollectorFixedEnvelope(t *testing.T) {
	// The conservative layouts cover all captures in the collector closures.
	type appendCutCapture struct {
		_    uintptr
		_, _ [][]Xseg
		_, _ map[int][]Xseg
		_    *PairCutsBudget
		_    context.Context //nolint:containedctx // This type measures a compiler capture.
	}
	type setRimCapture struct {
		_ uintptr
		_ *replayTable[string, float64]
		_ context.Context //nolint:containedctx // This type measures a compiler capture.
		_ map[string]float64
	}
	type callbackCapture struct {
		_ uintptr
		_ [2]*BoolMesh
		_ *[]TouchContact
		_ context.Context //nolint:containedctx // This type measures a compiler capture.
		_ *PairCutsBudget
		_ *[2]float64
		_ [10]unsafe.Pointer
	}
	bytes := unsafe.Sizeof(appendCutCapture{}) + unsafe.Sizeof(setRimCapture{}) + unsafe.Sizeof(callbackCapture{}) +
		7*unsafe.Sizeof([]int(nil)) + unsafe.Sizeof(float64(0)) + unsafe.Sizeof([2]float64{}) +
		unsafe.Sizeof(TouchContactError{}) + unsafe.Sizeof(BooleanExpectedError{})
	if bytes > replayCollectorFixedBytes {
		t.Fatalf("collector fixed captures need %d bytes, reserved %d", bytes, replayCollectorFixedBytes)
	}
	const fixedObjects = 3 + 7 + 2 + 1 + 1 + 1 + 1
	if fixedObjects > replayCollectorFixedRecords {
		t.Fatalf("collector fixed objects need %d records, reserved %d", fixedObjects, replayCollectorFixedRecords)
	}
}

func TestBrakePublicPairCutsRefusesReplayBudget(t *testing.T) {
	ctx := t.Context()
	meter := proofbound.NewBrakeReplayBudget()
	budget, err := NewReplayPairCutsBudget(meter)
	if err != nil {
		t.Fatal(err)
	}
	result, err := PairCuts(ctx, nil, nil, nil, budget)
	if result != nil || !errors.Is(err, decaderr.ErrUnsupported) {
		t.Fatalf("public replay admission: result=%t error=%v", result != nil, err)
	}
	work, records := meter.Usage()
	if work != 0 || records != 0 || meter.Bytes() != 0 {
		t.Fatalf("refused admission charged work=%d records=%d bytes=%d", work, records, meter.Bytes())
	}
}

var errBrakeContactErrorSink error

func TestBrakeCollectorContactFailureObjects(t *testing.T) {
	allocs := testing.AllocsPerRun(100, func() {
		errBrakeContactErrorSink = ExpectedBoolean(BooleanExpectedContact, &TouchContactError{What: touchCoplanar})
	})
	if allocs > 2 {
		t.Fatalf("contact refusal allocated %.0f objects, expected at most two", allocs)
	}
	var expected *BooleanExpectedError
	var contact *TouchContactError
	if !errors.As(errBrakeContactErrorSink, &expected) || !errors.As(errBrakeContactErrorSink, &contact) ||
		expected.Kind != BooleanExpectedContact || !errors.Is(errBrakeContactErrorSink, decaderr.ErrUnsupported) {
		t.Fatal("contact refusal lost its typed cause")
	}
}
