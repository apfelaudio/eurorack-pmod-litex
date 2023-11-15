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

fn draw_voice<D>(d: &mut D, sx: i32, sy: u32, ix: u32, voice: &Voice) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{

    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);
    let thin_stroke_grey = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x3))
        .stroke_width(1)
        .build();
    let character_style_h = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let title_y = 10u32;
    let box_h = 20u32;
    let box_y = title_y + 3u32 + box_h;

    let mut s: String<32> = String::new();

    // Voice box
    Rectangle::new(Point::new(sx, sy as i32), Size::new(30, box_y))
        .into_styled(thin_stroke_grey)
        .draw(d)?;

    // Title box
    Rectangle::new(Point::new(sx, sy as i32), Size::new(30, title_y))
        .into_styled(thin_stroke)
        .draw(d)?;

    // Channel title
    ufmt::uwrite!(&mut s, "CH {}", ix).ok();
    Text::with_alignment(
        &s,
        Point::new(sx+5, (sy as i32)+7),
        character_style_h,
        Alignment::Left,
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

    // Pitch text + box

    Text::new(
        &s,
        Point::new(sx+9, sy as i32 + 18),
        font_small_white,
    )
    .draw(d)?;

    Rectangle::new(Point::new(sx+2, sy as i32 + 11), Size::new(26, 11))
        .into_styled(stroke_idle)
        .draw(d)?;


    // LPF visualization

    let filter_x = sx+2;
    let filter_y = (sy as i32) + 23;
    let filter_w = 23;
    let filter_h = 7;
    let filter_skew = 2;
    let filter_pos: i32 = ((filter_w as f32) * voice.amplitude) as i32;

    Line::new(Point::new(filter_x,            filter_y),
              Point::new(filter_x+filter_pos, filter_y))
              .into_styled(stroke_gain)
              .draw(d)?;

    Line::new(Point::new(filter_x+filter_skew+filter_pos, filter_y+filter_h),
              Point::new(filter_x+filter_w+filter_skew,               filter_y+filter_h))
              .into_styled(stroke_gain)
              .draw(d)?;

    Line::new(Point::new(filter_x+filter_pos, filter_y),
              Point::new(filter_x+filter_pos+filter_skew, filter_y+filter_h))
              .into_styled(stroke_gain)
              .draw(d)?;


    Ok(())
}

fn draw_options<D>(d: &mut D, opts: &opt::Options) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{
    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let font_small_grey = MonoTextStyle::new(&FONT_4X6, Gray4::new(0x4));

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


    /*
    Text::with_alignment(
        "POLYPHONIZER",
        Point::new(d.bounding_box().center().x, 10),
        character_style,
        Alignment::Center,
    )
    .draw(d)?;
    */

    if opts.screen.value == opt::Screen::Adsr  {
        for (n_voice, voice) in voices.iter().enumerate() {
            if n_voice % 2 == 0 {
                draw_voice(d, 1, (1+35*n_voice/2) as u32,
                           n_voice as u32, voice)?;
            } else {
                draw_voice(d, 33, (1+35*(n_voice/2)) as u32,
                           n_voice as u32, voice)?;
            }
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

#[cfg(test)]
mod tests {
    use super::*;

    use image::{ImageBuffer, RgbImage, Rgb};
    use image::imageops::{rotate90, resize, FilterType};
    use midi_types::MidiMessage;

    struct FakeDisplay {
        img: RgbImage,
    }

    impl DrawTarget for FakeDisplay {
        type Color = Gray4;
        type Error = core::convert::Infallible;

        fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(coord, color) in pixels.into_iter() {
                if let Ok((x @ 0..=63, y @ 0..=255)) = coord.try_into() {
                    *self.img.get_pixel_mut(y, 63-x) = Rgb([
                        color.luma()<<5,
                        color.luma()<<5,
                        0
                    ]);
                }
            }

            Ok(())
        }
    }

    impl OriginDimensions for FakeDisplay {
        fn size(&self) -> Size {
            Size::new(64, 64)
        }
    }

    #[test]
    fn draw_screen() {
        let mut disp = FakeDisplay {
            img: ImageBuffer::new(256, 64)
        };
        let opts = opt::Options::new();
        let mut voice_manager = VoiceManager::new();
        let scope_samples = [0i16; 64];

        voice_manager.event(MidiMessage::NoteOn(0.into(), 70.into(), 64.into()), 0u32);
        voice_manager.tick(10u32, &opts);
        voice_manager.tick(50u32, &opts);
        voice_manager.tick(200u32, &opts);

        draw_main(&mut disp, opts, voice_manager.voices, &scope_samples, 1, 2).unwrap();

        let rot = rotate90(&disp.img);
        let rz = resize(&rot, 64*4, 256*4, FilterType::Nearest);

        rz.save("test.png").unwrap();
    }
}
