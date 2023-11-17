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

fn draw_title_box<D>(d: &mut D, title: &String<16>, top_left: Point, size: Size) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{
    let title_y = 10u32;
    let font_height = 7i32;

    let character_style_h = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);
    let thin_stroke_grey = PrimitiveStyleBuilder::new()
        .stroke_color(Gray4::new(0x3))
        .stroke_width(1)
        .build();

    // Outer box
    Rectangle::new(top_left, size)
        .into_styled(thin_stroke_grey)
        .draw(d)?;

    // Title box
    Rectangle::new(top_left, Size::new(size.width, title_y))
        .into_styled(thin_stroke)
        .draw(d)?;

    // Channel title
    Text::with_alignment(
        &title,
        Point::new(top_left.x + (size.width as i32)/2, top_left.y + font_height),
        character_style_h,
        Alignment::Center,
    )
    .draw(d)?;

    Ok(())
}

fn draw_voice<D>(d: &mut D, sx: i32, sy: u32, ix: u32, voice: &Voice) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{
    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);

    // Channel title
    let mut s: String<16> = String::new();
    ufmt::uwrite!(&mut s, "CH {}", ix).ok();
    draw_title_box(d, &s, Point::new(sx, sy as i32), Size::new(30, 33))?;

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

    let vy: usize = 152;

    let screen_hl = match (opts.view().selected(), opts.modify) {
        (None, _) => true,
        _ => false,
    };

    draw_title_box(d, &String::new(), Point::new(1, (vy-17) as i32), Size::new(62, 85))?;

    Text::with_alignment(
        opts.screen.value.into(),
        Point::new(40, (vy-10) as i32),
        if screen_hl { font_small_white } else { font_small_grey },
        Alignment::Left
    ).draw(d)?;

    Text::with_alignment(
        "OPTIONS: ",
        Point::new(4, (vy-10) as i32),
        if screen_hl { font_small_white } else { font_small_grey },
        Alignment::Left
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

fn draw_ms<D, S, C>(d: &mut D, name: &str, us: u32, pos: Point, style: S) -> Result<(), D::Error>
where
    D: DrawTarget<Color = C>,
    S: TextRenderer<Color = C>,
{
    let mut s: String<64> = String::new();
    ufmt::uwrite!(&mut s, "{}: {}.", name, us / 1_000u32).ok();
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

    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);

    for (n_voice, voice) in voices.iter().enumerate() {
        if n_voice % 2 == 0 {
            draw_voice(d, 1, (1+35*n_voice/2) as u32,
                       n_voice as u32, voice)?;
        } else {
            draw_voice(d, 33, (1+35*(n_voice/2)) as u32,
                       n_voice as u32, voice)?;
        }
    }

    draw_options(d, &opts)?;

    let mut s: String<16> = String::new();
    ufmt::uwrite!(&mut s, "STATS").ok();
    draw_title_box(d, &s, Point::new(1, 222), Size::new(62, 32))?;

    draw_ms(d, "main", trace_main_len_us,
            Point::new(5, 249), font_small_white)?;
    draw_ms(d, "irq0", irq0_len_us,
            Point::new(5, 239), font_small_white)?;

    {
        let mut s: String<16> = String::new();
        ufmt::uwrite!(&mut s, "SCOPE").ok();
        draw_title_box(d, &s, Point::new(1, 71), Size::new(62, 62))?;

        let mut points: [Point; 60] = [Point::new(0, 0); 60];
        for (n, point) in points.iter_mut().enumerate() {
            if n < scope_samples.len() {
                point.x = 2 + n as i32;
                point.y = 105 + (scope_samples[n] >> 10) as i32;
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
