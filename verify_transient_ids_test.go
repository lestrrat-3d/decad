package decad

// CI: isolated-tests

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// TestVerifyOverlappingPairsRace runs one Verify over four mutually
// overlapping prisms. Its pair workers build transient bodies at once; under
// -race they must not write one shared id counter.
func TestVerifyOverlappingPairsRace(t *testing.T) {
	doc := overlappingBoxDocument(t, 4)
	_, err := doc.Verify(t.Context())
	require.NoError(t, err)
}

// TestVerifyConcurrentWithVerify runs two Verify calls on one document at
// once, which api-design section 12 promises is safe.
func TestVerifyConcurrentWithVerify(t *testing.T) {
	doc := overlappingBoxDocument(t, 3)
	runConcurrently(t, 2, func() error {
		_, err := doc.Verify(t.Context())
		return err
	})
}

// TestVerifyLeavesCountersUnchanged checks that Verify moves neither the
// level counter nor the curve counter, so the ids of bodies built after it do
// not depend on it.
func TestVerifyLeavesCountersUnchanged(t *testing.T) {
	doc := overlappingBoxDocument(t, 3)
	level, curve, producer := doc.n.NextLevel, doc.n.NextCurve, doc.n.NextProducer
	_, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, level, doc.n.NextLevel, "level counter")
	require.Equal(t, curve, doc.n.NextCurve, "curve counter")
	require.Equal(t, producer, doc.n.NextProducer, "producer counter")
}

// TestCircleWitnessReadsRaceWithCommit replays a circle source chain while
// another goroutine commits new circle extrudes, which write the witness maps.
func TestCircleWitnessReadsRaceWithCommit(t *testing.T) {
	doc := New()
	body := internalDiscBody(t, doc, 3, 2)
	done := make(chan struct{})
	go func() {
		defer close(done)
		for range 300 {
			internalDiscBody(t, doc, 2, 1)
		}
	}()
	for range 300 {
		_, err := captureCircleSource(t.Context(), doc, body, proofbound.NewCircleReplayBudget())
		require.NoError(t, err)
	}
	<-done
}
