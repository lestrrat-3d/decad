package dynamics

import "github.com/lestrrat-3d/decad"

// TracePairProof is one scheduled pair's certificate in a trace slice, read
// by the external schedule tests: the pair's sweep, or the swept-box
// exclusion that made no sweep necessary.
type TracePairProof struct {
	Pair     BodyPair
	BoxClear bool
	Sweep    *decad.SweepReport
}

// TraceSliceProofs returns the scheduled-pair certificates of each slice of
// a trace of a world of four or more bodies, in canonical pair order.
func TraceSliceProofs(tr Trace) [][]TracePairProof {
	out := make([][]TracePairProof, len(tr.slices))
	for i, slice := range tr.slices {
		for _, proof := range slice.proofs {
			out[i] = append(out[i], TracePairProof{Pair: slice.from.world.bodyPair(slice.from.world.pairs[proof.pair]),
				BoxClear: proof.boxClear, Sweep: proof.sweep})
		}
	}
	return out
}
