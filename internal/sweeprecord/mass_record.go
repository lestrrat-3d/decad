package sweeprecord

import "github.com/lestrrat-3d/decad/internal/massmoment"

// MassRecord presents the sweep's certified span reductions to the mass
// integrator.
func (sp Payload) MassRecord() massmoment.SweepRecord {
	record := massmoment.SweepRecord{Arc: sp.Arc}
	if len(sp.Spans) == 0 {
		if sp.Arc {
			record.Revolve = sp.Revolve.MassRecord()
		} else {
			record.Prism = sp.Prism.MassRecord()
		}
		return record
	}
	record.Spans = make([]massmoment.SweepSpanRecord, len(sp.Spans))
	for i, span := range sp.Spans {
		record.Spans[i] = massmoment.SweepSpanRecord{Arc: span.Arc}
		if span.Arc {
			record.Spans[i].Revolve = span.Revolve.MassRecord()
		} else {
			record.Spans[i].Prism = span.Prism.MassRecord()
		}
	}
	return record
}
