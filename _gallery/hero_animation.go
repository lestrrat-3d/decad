package main

import (
	"context"
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strconv"
	"time"

	"github.com/lestrrat-3d/units"

	"github.com/lestrrat-3d/kinetograph"
)

const (
	heroAnimationLength = 16 * time.Second
	heroHoldStart       = 4 * time.Second
	heroExitStart       = heroHoldStart + 10*time.Second
	heroLetterMove      = 900 * time.Millisecond
	heroLetterStagger   = 150 * time.Millisecond
)

// heroAnimationScript drops the letters onto the fixed hero camera's plate,
// sweeps the word lamp, holds the finished logo for ten seconds, then lifts
// the letters out in reverse order. Its first and last poses are the same.
func heroAnimationScript() Script {
	mm := units.Millimeters
	s := Script{Tracks: []Track{
		{Name: "word.dolly", Initial: mm(0)},
		{Name: "lamp.word.slide", Initial: mm(0)},
		{Name: "lamp.word.intensity", Initial: units.Scalar(0)},
	}}
	letters := decadLetters()
	for i := range letters {
		name := letterTrack(i)
		s.Tracks = append(s.Tracks, Track{Name: name, Initial: mm(0)})
		start := 300*time.Millisecond + time.Duration(i)*heroLetterStagger
		exit := heroExitStart + time.Duration(len(letters)-1-i)*heroLetterStagger
		s.Moves = append(s.Moves,
			Move{Track: name, Start: start, End: start + heroLetterMove,
				To: mm(letterTravel), Ease: kinetograph.EaseOut},
			Move{Track: name, Start: exit, End: exit + heroLetterMove,
				To: mm(0), Ease: kinetograph.EaseIn},
		)
	}
	s.Moves = append(s.Moves,
		Move{Track: "lamp.word.slide", Start: 2500 * time.Millisecond, End: heroHoldStart,
			To: mm(wordLampTravel), Ease: kinetograph.Linear},
		Move{Track: "lamp.word.intensity", Start: 2500 * time.Millisecond, End: 2800 * time.Millisecond,
			To: units.Scalar(wordLampIntensity), Ease: kinetograph.EaseOut},
		Move{Track: "lamp.word.intensity", Start: 3700 * time.Millisecond, End: heroHoldStart,
			To: units.Scalar(0), Ease: kinetograph.EaseIn},
	)
	return s
}

// runHero renders the README hero's frames and prints the ffmpeg command that
// turns them into a looping GIF. -smoke renders only its first, small frame.
func runHero(ctx context.Context, args []string, stdout io.Writer) error {
	var opts clipOptions
	var gif string
	fs := flag.NewFlagSet("hero", flag.ContinueOnError)
	fs.StringVar(&opts.out, "out", filepath.Join("out", "hero"), "directory to write the frames to")
	fs.StringVar(&gif, "gif", "", "GIF destination (default: docs/images/hero.gif)")
	fs.IntVar(&opts.width, "width", 900, "frame width in pixels (even)")
	fs.IntVar(&opts.height, "height", 506, "frame height in pixels (even)")
	fs.IntVar(&opts.fps, "fps", 16, "frames per second (even)")
	fs.IntVar(&opts.workers, "workers", min(runtime.NumCPU(), 8), "frames rendered at once")
	fs.BoolVar(&opts.smoke, "smoke", false, "render only the first frame at 160x90")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() > 0 {
		return fmt.Errorf("hero: unexpected argument %q", fs.Arg(0))
	}
	if opts.smoke {
		opts.width, opts.height = smokeWidth, smokeHeight
	}
	if err := validateFormat(opts.fps, opts.width, opts.height); err != nil {
		return err
	}
	if opts.workers < 1 {
		return fmt.Errorf("-workers %d: must be at least 1", opts.workers)
	}
	if gif == "" && !opts.smoke {
		root, err := imagesRoot("")
		if err != nil {
			return err
		}
		gif = filepath.Join(root, "hero.gif")
	}
	if !opts.smoke {
		if err := os.MkdirAll(filepath.Dir(gif), 0o755); err != nil {
			return fmt.Errorf("create GIF directory: %w", err)
		}
	}
	shot := Shot{Name: "hero", From: 0, To: heroAnimationLength, Build: wordmarkTake}
	if err := renderShot(ctx, shot, heroAnimationScript(), opts); err != nil {
		return err
	}
	if !opts.smoke {
		fmt.Fprintln(stdout, heroGIFCommand(opts.fps, opts.out, gif))
	}
	return nil
}

// heroGIFCommand encodes the rendered frames with one 128-colour palette.
func heroGIFCommand(fps int, frames, gif string) command {
	const palette = "[0:v]split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];" +
		"[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle[v]"
	return command{
		{"ffmpeg", "-y", "-loglevel", "error"},
		{"-framerate", strconv.Itoa(fps), "-i", filepath.Join(frames, "hero_%06d.png")},
		{"-filter_complex", palette},
		{"-map", "[v]", "-loop", "0", gif},
	}
}
