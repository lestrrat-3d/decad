package momentinput

import "slices"

// normalizeReconstructionWeights rewrites every all-equal NURBS weight vector
// in a record to ones, for the sketch reconstruction only.
//
// Equal weights CANCEL in the homogeneous quotient, so the normalized segment
// names the very same curve — the exact integration reads the recorded weights
// and is untouched by this. What changes is the magnitude sketch's own evaluator
// works in: it squares the homogeneous denominator to differentiate, so weights
// past about sqrt(MaxFloat64) overflow it and the reconstruction yields no valid
// profile at all. A record whose weights are all 1e300 is then refused as a
// region that does not close, while the identical curve at weight 1 measures
// fine. The record is Tier A and owes an answer, so the reconstruction is asked
// about the representable spelling of the same curve.
//
// The normalized record is what the later record comparison runs against too: sketch
// records the entity it was given, so a candidate carries the normalized weights
// and must be matched against a record carrying them as well.
func normalizeReconstructionWeights(record Profile) Profile {
	loops := append([]LoopRecord{record.Outer}, record.Holes...)
	out := make([]LoopRecord, len(loops))
	changed := false
	for loopIndex, loop := range loops {
		out[loopIndex] = loop
		for segmentIndex, segment := range loop.Segments {
			seg, ok := segment.(NURBSSeg)
			if !ok || !equalNURBSWeights(seg.Weights) || seg.Weights[0] == 1 {
				continue
			}
			if !changed {
				for i, source := range loops {
					out[i].Segments = slices.Clone(source.Segments)
				}
				changed = true
			}
			ones := make([]float64, len(seg.Weights))
			for i := range ones {
				ones[i] = 1
			}
			seg.Weights = ones
			out[loopIndex].Segments[segmentIndex] = seg
		}
	}
	if !changed {
		return record
	}
	return Profile{Outer: out[0], Holes: out[1:]}
}

// equalNURBSWeights reports whether every weight is the same value — the
// non-rational (Tier A) condition the exact conversion also tests.
func equalNURBSWeights(weights []float64) bool {
	if len(weights) == 0 {
		return false
	}
	for _, weight := range weights {
		if weight != weights[0] {
			return false
		}
	}
	return true
}
