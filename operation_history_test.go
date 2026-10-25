package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestOperationHistorySnapshotAndFaceCoverage(t *testing.T) {
	doc := New()
	empty := doc.OperationHistory()
	require.Empty(t, empty.Operations())

	source := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	snapshot := doc.OperationHistory()
	created := snapshot.Operations()
	require.Len(t, created, 1)
	require.Equal(t, OperationExtrude, created[0].Kind())
	require.Equal(t, []*Body{source}, created[0].Outputs())

	placed, err := Transform(t.Context(), source, r3.Identity())
	require.NoError(t, err)
	transformed, err := Transform(t.Context(), placed, r3.Identity())
	require.NoError(t, err)
	latest := doc.OperationHistory()
	require.Len(t, snapshot.Operations(), 1)
	require.Len(t, latest.Operations(), 3)

	producer, err := snapshot.Producer(source)
	require.NoError(t, err)
	require.Same(t, created[0], producer)
	future, err := snapshot.Producer(placed)
	require.ErrorIs(t, err, ErrNoMatch)
	require.Nil(t, future)
	futureConsumers, err := snapshot.Consumers(placed)
	require.ErrorIs(t, err, ErrNoMatch)
	require.Nil(t, futureConsumers)

	consumers, err := latest.Consumers(source)
	require.NoError(t, err)
	require.Len(t, consumers, 1)
	require.Equal(t, OperationTransform, consumers[0].Kind())

	transform := latest.Operations()[1]
	inputs := transform.Inputs()
	inputs[0].Role = "changed"
	require.Equal(t, "source", transform.Inputs()[0].Role)
	outputs := transform.Outputs()
	outputs[0] = nil
	require.Same(t, placed, transform.Outputs()[0])

	face := source.Faces()[0]
	correspondence, err := transform.FaceSuccessors(face)
	require.NoError(t, err)
	require.Equal(t, HistoryUnknown, correspondence.Coverage())
	require.Empty(t, correspondence.Faces())
	_, err = transform.FaceSuccessors(transformed.Faces()[0])
	require.ErrorIs(t, err, ErrNoMatch)

	copied, err := Copy(t.Context(), transformed)
	require.NoError(t, err)
	copyHistory := doc.OperationHistory()
	copyProducer, err := copyHistory.Producer(copied)
	require.NoError(t, err)
	require.Equal(t, OperationCopy, copyProducer.Kind())
	require.Equal(t, []OperationInput{{Body: transformed, Role: operationRoleSource, Use: InputReferenced}}, copyProducer.Inputs())
	copyConsumers, err := copyHistory.Consumers(transformed)
	require.NoError(t, err)
	require.Empty(t, copyConsumers)

	foreign := New()
	foreignBody := internalBoxBody(t, foreign, 0, 0, 1, 1, 1)
	_, err = latest.Producer(foreignBody)
	require.ErrorIs(t, err, ErrForeignBody)
	_, err = latest.Consumers(foreignBody)
	require.ErrorIs(t, err, ErrForeignBody)
}

func TestPrivateConstructionKeepsBodiesAndProducerNumbersLocal(t *testing.T) {
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 2, 2, 2)
	beforeBodies := doc.Bodies()
	beforeProducer := doc.nextProducerID()
	ctx, capture := privateConstructionContext(t.Context())
	ref := doc.producerID(ctx)
	require.Equal(t, beforeProducer, ref)
	require.NoError(t, doc.commitOperationSpanIn(ctx, nil, OperationExtrude, nil, body, 1))
	require.Equal(t, beforeProducer, doc.nextProducerID())
	require.Equal(t, beforeProducer+1, doc.producerID(ctx))
	require.Equal(t, beforeBodies, doc.Bodies())
	require.Len(t, doc.OperationHistory().Operations(), 1)
	require.Equal(t, producerID(1), capture.producerSpan(doc))
}

func TestOperationHistoryRemovePublishesZeroOutput(t *testing.T) {
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 2, 2, 2)
	require.NoError(t, Remove(body))

	history := doc.OperationHistory()
	operations := history.Operations()
	require.Len(t, operations, 2)
	remove := operations[1]
	require.Equal(t, OperationRemove, remove.Kind())
	require.Empty(t, remove.Outputs())
	require.Equal(t, []OperationInput{{Body: body, Role: "body", Use: InputConsumed}}, remove.Inputs())
	consumers, err := history.Consumers(body)
	require.NoError(t, err)
	require.Equal(t, []*Operation{remove}, consumers)
	require.Empty(t, doc.Bodies())
	for _, face := range body.Faces() {
		mapping, err := remove.FaceSuccessors(face)
		require.NoError(t, err)
		require.Equal(t, HistoryComplete, mapping.Coverage())
		require.Empty(t, mapping.Faces())
	}
	_, err = (*OperationHistory)(nil).Producer(body)
	require.ErrorIs(t, err, ErrDegenerate)
	_, err = (*Operation)(nil).FaceSuccessors(body.Faces()[0])
	require.ErrorIs(t, err, ErrDegenerate)

	unstitchDoc := New()
	stitched := internalBoxBody(t, unstitchDoc, 0, 0, 2, 2, 2)
	sheets, err := Unstitch(t.Context(), stitched)
	require.NoError(t, err)
	require.Len(t, sheets, 6)
	unstitch, err := unstitchDoc.OperationHistory().Producer(sheets[0])
	require.NoError(t, err)
	require.Equal(t, OperationUnstitch, unstitch.Kind())
	require.Equal(t, sheets, unstitch.Outputs())
	for i, face := range stitched.Faces() {
		mapping, err := unstitch.FaceSuccessors(face)
		require.NoError(t, err)
		require.Equal(t, HistoryComplete, mapping.Coverage())
		require.Equal(t, []*Face{sheets[i].Faces()[0]}, mapping.Faces())
	}
}
