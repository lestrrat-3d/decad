package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// volumeCase builds one operand pair in a fresh document. Calling make twice gives two twin documents
// whose pairs are built the same way.
type volumeCase struct {
	name string
	// outcome is what Intersect returns for the pair: "ok", "empty", "empty-shift" or "unsupported".
	outcome string
	make    func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body)
}

func intersectionVolumeCases() []volumeCase {
	return []volumeCase{
		{"overlapping prisms", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10), boxBodyAtZ(t, doc, 5, 5, 15, 15, 5, 10)
		}},
		{"coplanar co-directional prisms", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, boxBody(t, doc, 0, 0, 10, 10, 10), boxBody(t, doc, 5, 5, 15, 15, 10)
		}},
		{"far-apart boxes", "empty", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, boxBody(t, doc, 0, 0, 10, 10, 10), boxBody(t, doc, 100, 100, 110, 110, 10)
		}},
		{"face-touching boxes, gap shift", "empty-shift", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			a, b := gapBoxes(t, 0.5, 0, 10, 10, 10, 10)
			return nil, a, b
		}},
		{"face-touching boxes, exact only", "unsupported", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			a, b := gapBoxes(t, 0.5, 0, 10, 10, 10, 10, decad.ExactOnly())
			return nil, a, b
		}},
		{"cylinder through a box", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			return doc, roundTrip(t, boxBodyAtZ(t, doc, 0, 0, 20, 20, 0, 10)), circlePrismAtZ(t, doc, 10, 10, 4, -2, 14)
		}},
		{"orthogonal mesh solids", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			doc := decad.New()
			a := roundTrip(t, boxBodyAtZ(t, doc, 0, 0, 10, 10, 0, 10))
			b := roundTrip(t, boxBodyAtZ(t, doc, 4, 4, 14, 14, 3, 10))
			return doc, a, b
		}},
		{"regroup after a mesh cut", "ok", func(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
			sc := newRetentionScene()
			box, t1, c := sc.roundedBox(t), sc.tool(t, 20), regroupSlabC(t, sc)
			first, err := decad.Cut(t.Context(), box, t1)
			require.NoError(t, err)
			return sc.doc, first, c
		}},
	}
}

// build makes the case's pair and the document that holds it.
func (c volumeCase) build(t *testing.T) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc, a, b := c.make(t)
	if doc == nil {
		doc = a.Document()
	}
	return doc, a, b
}

// requireSameOutcome requires the query's outcome and Intersect's to agree: nil-ness, sentinels, the boolean
// error Code, message and gap shift, and the volume's value, exactness and bound by bits.
func requireSameOutcome(t *testing.T, queried decad.Measurement, qerr error, got *decad.Body, ierr error) {
	t.Helper()
	if ierr != nil {
		require.Error(t, qerr)
		for _, sentinel := range []error{decad.ErrBooleanFailed, decad.ErrUnsupported, decad.ErrDegenerate} {
			require.Equal(t, errors.Is(ierr, sentinel), errors.Is(qerr, sentinel), "sentinel %v", sentinel)
		}
		var ib, qb *decad.BooleanError
		require.Equal(t, errors.As(ierr, &ib), errors.As(qerr, &qb))
		if ib != nil {
			require.Equal(t, ib.Code, qb.Code)
			require.Equal(t, ib.Error(), qb.Error())
			require.Equal(t, ib.OperationDiagnostics(), qb.OperationDiagnostics())
		}
		return
	}
	require.NoError(t, qerr)
	want, err := got.Volume()
	require.NoError(t, err)
	requireSameMeasurement(t, want, queried)
}

func requireSameMeasurement(t *testing.T, want, got decad.Measurement) {
	t.Helper()
	require.Equal(t, want.Exactness, got.Exactness)
	require.Equal(t, math.Float64bits(want.Value.Base()), math.Float64bits(got.Value.Base()))
	require.Equal(t, math.Float64bits(want.Bound.Base()), math.Float64bits(got.Bound.Base()))
}

// requireOutcomeKind requires err to be the outcome a case names.
func requireOutcomeKind(t *testing.T, want string, err error) {
	t.Helper()
	var be *decad.BooleanError
	switch want {
	case "ok":
		require.NoError(t, err)
	case "empty":
		require.ErrorAs(t, err, &be)
		require.Equal(t, decad.BooleanEmpty, be.Code)
	case "empty-shift":
		require.ErrorAs(t, err, &be)
		require.Equal(t, decad.BooleanEmpty, be.Code)
		shifted := false
		for _, d := range be.OperationDiagnostics() {
			shifted = shifted || d.Change != nil
		}
		require.True(t, shifted, "the empty result reports the gap shift")
	case "unsupported":
		require.ErrorIs(t, err, decad.ErrUnsupported)
	default:
		require.Failf(t, "bad outcome", "%q", want)
	}
}
