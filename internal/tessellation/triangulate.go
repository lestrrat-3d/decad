package tessellation

// IsBridgeStub reports whether the zero-area corner (ia, ib, ic) is a bridge
// stub safe to collapse — the tip or residue of a bridge's zero-width channel,
// where an index coincides among the three corner vertices. The spike apex M
// carries the SAME bridge anchor index P on both sides (ia == ic); collapsing it
// drops the reverse edges P→M and M→P and nothing else. The doubled anchor left
// behind carries an adjacent identical index (ia == ib or ib == ic); collapsing
// it drops only a zero-length self edge. A collinear corner whose three vertices
// are DISTINCT indices is NOT a stub — it is a genuine boundary corner whose two
// edges (one possibly from another hole sharing this v-line) must be kept.
func IsBridgeStub(ia, ib, ic int) bool {
	return ia == ic || ia == ib || ib == ic
}
