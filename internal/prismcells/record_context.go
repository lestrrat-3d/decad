package prismcells

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectioncert"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

// RecordProfileContext makes RecordProfile's own internal re-arrangement
// (sketchrecord.AuthenticateProfile's fresh s.Profiles() call) observable to a
// caller's context, the same way prismcells.ProfilesContext wraps the FIRST
// arrangement. The scene is already capped by prismcells.MaxArrangementSegments, so
// this second pass over it stays bounded too. The returned PlaneRecord is not
// read here — the caller keeps operand A's own frame/xform (§4.1) — so only
// the ProfileRecord and error are surfaced.
func RecordProfileContext(ctx context.Context, s *sketch.Sketch, p *sketch.Profile) (momentinput.Profile, error) {
	return profileRecordContext(ctx, func() (momentinput.Profile, error) {
		profile, _, _, err := sectioncert.RecordProfile(s, p)
		return profile, err
	})
}

// RecordArrangedProfileContext is for a profile returned by the private
// scene's first Profiles call. The caller owns that scene and selects from its
// returned slice, so no second authentication arrangement is needed.
func RecordArrangedProfileContext(ctx context.Context, p *sketch.Profile) (momentinput.Profile, error) {
	return profileRecordContext(ctx, func() (momentinput.Profile, error) {
		return recordArrangedProfile(p)
	})
}

func profileRecordContext(ctx context.Context, record func() (momentinput.Profile, error)) (momentinput.Profile, error) {
	if err := ctx.Err(); err != nil {
		return momentinput.Profile{}, err
	}
	type prismRecordResult struct {
		profile momentinput.Profile
		err     error
	}
	done := make(chan prismRecordResult)
	go func() {
		profile, err := record()
		done <- prismRecordResult{profile: profile, err: err}
	}()
	select {
	case result := <-done:
		return result.profile, result.err
	case <-ctx.Done():
		<-done
		return momentinput.Profile{}, ctx.Err()
	}
}

// recordArrangedProfile records a profile from a fresh sketch arrangement.
// Callers that skip sketchrecord.AuthenticateProfile must own the sketch and
// pass a profile returned by that same arrangement. The record walk checks
// each fragment's TExact claim, range, and loop closure.
func recordArrangedProfile(p *sketch.Profile) (momentinput.Profile, error) {
	outer, holes, err := sketchrecord.RecordProfileLoops(p)
	if err != nil {
		return momentinput.Profile{}, err
	}
	return momentinput.Profile{Outer: outer, Holes: holes}, nil
}
