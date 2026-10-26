package decad

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
)

func TestCombineToolsAdditionalConsumptionIsAtomic(t *testing.T) {
	t.Parallel()
	for _, mode := range []string{"success", "reject", "cancel"} {
		t.Run(mode, func(t *testing.T) {
			doc := New()
			source := internalBoxBody(t, doc, 0, 0, 3, 3, 3)
			target, err := Copy(t.Context(), source)
			require.NoError(t, err)
			tool := internalBoxBody(t, doc, 1, 1, 2, 2, 3)
			defer removeTools([]*Body{tool})
			beforeHistory := len(doc.OperationHistory().Operations())
			beforeProducer := doc.nextProducerID()
			ctx, cancel := context.WithCancel(t.Context())
			defer cancel()
			_, capture := privateConstructionContext(ctx)
			rejected := errors.New("reject the completed cut")
			accept := func(result *Body) error {
				volume, err := result.Volume()
				require.NoError(t, err)
				require.InDelta(t, 24, volume.Value.Base(), volume.Bound.Base())
				require.NoError(t, doc.requireLive(source))
				require.NoError(t, doc.requireLive(target))
				if mode == "reject" {
					return rejected
				}
				if mode == "cancel" {
					cancel()
				}
				return nil
			}
			result, err := combineTools(ctx, doc, OperationThread, target, meshbool.OpCut,
				[]*Body{tool}, false, accept, capture,
				OperationInput{Body: source, Role: operationRoleSource, Use: InputConsumed},
				OperationInput{Body: source, Role: operationRoleOperand, Use: InputConsumed})
			if mode != "success" {
				want := rejected
				if mode == "cancel" {
					want = context.Canceled
				}
				require.ErrorIs(t, err, want)
				require.Nil(t, result)
				require.NoError(t, doc.requireLive(source))
				require.NoError(t, doc.requireLive(target))
				require.Len(t, doc.Bodies(), 3)
				require.Len(t, doc.OperationHistory().Operations(), beforeHistory)
				require.Equal(t, beforeProducer, doc.nextProducerID())
				return
			}
			require.NoError(t, err)
			require.ErrorIs(t, doc.requireLive(source), ErrRetiredBody)
			require.ErrorIs(t, doc.requireLive(target), ErrRetiredBody)
			require.NoError(t, doc.requireLive(result))
			require.NoError(t, doc.requireLive(tool))
			require.Len(t, doc.Bodies(), 2)
			history := doc.OperationHistory()
			require.Len(t, history.Operations(), beforeHistory+1)
			producer, err := history.Producer(result)
			require.NoError(t, err)
			require.Len(t, producer.Inputs(), 3)
			for _, body := range []*Body{source, target} {
				consumers, err := history.Consumers(body)
				require.NoError(t, err)
				require.Len(t, consumers, 1)
				require.True(t, consumers[0] == producer)
			}
		})
	}
}
