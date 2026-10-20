package provenance

// DedupProducers deduplicates recorded stop refs, preserving first occurrence,
// and returns nil when there are no dependencies so a record carries no Inputs
// at all.
func DedupProducers(refs []Producer) []Producer {
	if len(refs) == 0 {
		return nil
	}
	seen := make(map[Producer]struct{}, len(refs))
	out := make([]Producer, 0, len(refs))
	for _, r := range refs {
		if _, ok := seen[r]; ok {
			continue
		}
		seen[r] = struct{}{}
		out = append(out, r)
	}
	return out
}
