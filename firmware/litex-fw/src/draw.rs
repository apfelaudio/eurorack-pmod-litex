use heapless::String;

use embedded_graphics::{
    pixelcolor::{Gray4, GrayColor},
    primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, Line, Polyline},
    mono_font::{ascii::FONT_4X6, ascii::FONT_5X7, MonoTextStyle},
    prelude::*,
    text::{Alignment, Text, renderer::TextRenderer},
};

use crate::voice::*;
use crate::opt;

fn draw_voice<D>(d: &mut D, sy: u32, ix: u32, voice: &Voice) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{

    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);
    let thin_stroke_grey = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x3))
        .stroke_width(1)
        .build();
    let character_style_h = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let title_y = 10u32;
    let box_h = 20u32;
    let box_y = title_y + 3u32 + box_h;

    let mut s: String<32> = String::new();

    Rectangle::new(Point::new(2, sy as i32), Size::new(60, box_y))
        .into_styled(thin_stroke_grey)
        .draw(d)?;

    Rectangle::new(Point::new(2, sy as i32), Size::new(60, title_y))
        .into_styled(thin_stroke)
        .draw(d)?;

    ufmt::uwrite!(&mut s, "CH {}", ix).ok();

    Text::with_alignment(
        &s,
        Point::new(d.bounding_box().center().x, (sy as i32)+7),
        character_style_h,
        Alignment::Center,
    )
    .draw(d)?;


    let mut stroke_gain = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x1))
        .stroke_width(1)
        .build();

    let mut stroke_idle = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x1))
        .stroke_width(1)
        .build();

    s.clear();
    if voice.state != VoiceState::Idle {
        let semitones = voice.note as i32 - 60i32;
        if semitones > 0 {
            ufmt::uwrite!(&mut s, "+").ok();
        }
        ufmt::uwrite!(&mut s, "{}", semitones).ok();

        stroke_gain = PrimitiveStyleBuilder::new()
            .stroke_color(Gray4::new((15f32 * voice.amplitude) as u8))
            .stroke_width(1)
            .build();

        stroke_idle = PrimitiveStyleBuilder::new()
            .stroke_color(Gray4::WHITE)
            .stroke_width(1)
            .build();
    }

    let filter_pos: i32 = (20f32 * voice.amplitude) as i32;

    Line::new(Point::new(38, sy as i32 + 16),
              Point::new(40+filter_pos-2, sy as i32 + 16))
              .into_styled(stroke_gain)
              .draw(d)?;

    Line::new(Point::new(40+filter_pos, sy as i32 + 24),
              Point::new(55, sy as i32 + 24))
              .into_styled(stroke_gain)
              .draw(d)?;

    Line::new(Point::new(40+filter_pos-2, sy as i32 + 16),
              Point::new(40+filter_pos, sy as i32 + 24))
              .into_styled(stroke_gain)
              .draw(d)?;

    Rectangle::new(Point::new(7, sy as i32 + 15), Size::new(19, 11))
        .into_styled(stroke_idle)
        .draw(d)?;

    Text::new(
        &s,
        Point::new(9, sy as i32 + 22),
        character_style_h,
    )
    .draw(d)?;


    Ok(())
}

fn draw_options<D>(d: &mut D, opts: &opt::Options) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{
    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let font_small_grey = MonoTextStyle::new(&FONT_4X6, Gray4::new(0x4));

    // Should always succeed if the above CS runs.
    let opts_view = opts.view().options();

    let vy: usize = 205;

    // Draw the current screen text
    Text::with_alignment(
        opts.screen.value.into(),
        Point::new(5, (vy-10) as i32),
        match (opts.view().selected(), opts.modify) {
            (None, _) => font_small_white,
            _ => font_small_grey,
        },
        Alignment::Left,
    ).draw(d)?;

    for (n, opt) in opts_view.iter().enumerate() {
        let mut font = font_small_grey;
        if let Some(n_selected) = opts.view().selected() {
            if n_selected == n {
                font = font_small_white;
                if opts.modify {
                    Text::with_alignment(
                        "-",
                        Point::new(62, (vy+10*n) as i32),
                        font,
                        Alignment::Left,
                    ).draw(d)?;
                }
            }
        }
        Text::with_alignment(
            opt.name(),
            Point::new(5, (vy+10*n) as i32),
            font,
            Alignment::Left,
        ).draw(d)?;
        Text::with_alignment(
            &opt.value(),
            Point::new(60, (vy+10*n) as i32),
            font,
            Alignment::Right,
        ).draw(d)?;
    }

    Ok(())
}

fn draw_ms<D, S, C>(d: &mut D, us: u32, pos: Point, style: S) -> Result<(), D::Error>
where
    D: DrawTarget<Color = C>,
    S: TextRenderer<Color = C>,
{
    let mut s: String<64> = String::new();
    ufmt::uwrite!(&mut s, "{}.", us / 1_000u32).ok();
    ufmt::uwrite!(&mut s, "{}ms\n", us % 1_000u32).ok();
    Text::with_alignment(
        &s,
        pos,
        style,
        Alignment::Left,
    )
    .draw(d)?;

    Ok(())
}

pub fn draw_main<D>(d: &mut D,
                opts: opt::Options,
                voices: [Voice; N_VOICES],
                scope_samples: &[i16],
                irq0_len_us: u32,
                trace_main_len_us: u32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{

    let character_style = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);


    Text::with_alignment(
        "POLYPHONIZER",
        Point::new(d.bounding_box().center().x, 10),
        character_style,
        Alignment::Center,
    )
    .draw(d)?;

    if opts.screen.value == opt::Screen::Adsr  {
        for (n_voice, voice) in voices.iter().enumerate() {
            draw_voice(d, (55+37*n_voice) as u32,
                       n_voice as u32, voice)?;
        }
    }

    draw_options(d, &opts)?;

    draw_ms(d, trace_main_len_us,
            Point::new(5, 255), character_style)?;
    draw_ms(d, irq0_len_us,
            Point::new(5, 245), character_style)?;

    if opts.screen.value == opt::Screen::Scope  {
        let mut points: [Point; 64] = [Point::new(0, 0); 64];
        for (n, point) in points.iter_mut().enumerate() {
            if n < scope_samples.len() {
                point.x = n as i32;
                point.y = 30 + (scope_samples[n] >> 10) as i32;
            }
        }
        Polyline::new(&points)
            .into_styled(thin_stroke)
            .draw(d)?;
    }

    Ok(())
}
