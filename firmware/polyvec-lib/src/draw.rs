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
        title,
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
    draw_title_box(d, &s, Point::new(sx, sy as i32), Size::new(31, 32))?;

    let mut stroke_gain = PrimitiveStyleBuilder::new()
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
    }

    // Pitch text + box

    Text::new(
        &s,
        Point::new(sx+9, sy as i32 + 16),
        font_small_white,
    )
    .draw(d)?;

    // LPF visualization

    let filter_x = sx+2;
    let filter_y = (sy as i32) + 19;
    let filter_w = 23;
    let filter_h = 8;
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

    let vx: i32 = 128+64;
    let vy: usize = 17;

    let screen_hl = match (opts.view().selected(), opts.modify) {
        (None, _) => true,
        _ => false,
    };

    draw_title_box(d, &String::new(), Point::new(vx, (vy-17) as i32), Size::new(64, 64))?;

    Text::with_alignment(
        opts.screen.value.into(),
        Point::new(vx+40, (vy-10) as i32),
        if screen_hl { font_small_white } else { font_small_grey },
        Alignment::Left
    ).draw(d)?;

    Text::with_alignment(
        "OPTIONS: ",
        Point::new(vx+4, (vy-10) as i32),
        if screen_hl { font_small_white } else { font_small_grey },
        Alignment::Left
    ).draw(d)?;

    let vx = vx-2;

    for (n, opt) in opts_view.iter().enumerate() {
        let mut font = font_small_grey;
        if let Some(n_selected) = opts.view().selected() {
            if n_selected == n {
                font = font_small_white;
                if opts.modify {
                    Text::with_alignment(
                        "-",
                        Point::new(vx+62, (vy+7*n) as i32),
                        font,
                        Alignment::Left,
                    ).draw(d)?;
                }
            }
        }
        Text::with_alignment(
            opt.name(),
            Point::new(vx+5, (vy+7*n) as i32),
            font,
            Alignment::Left,
        ).draw(d)?;
        Text::with_alignment(
            &opt.value(),
            Point::new(vx+60, (vy+7*n) as i32),
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
                touch: &[u8],
                irq0_len_us: u32,
                trace_main_len_us: u32) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{

    let font_small_white = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    let thin_stroke = PrimitiveStyle::with_stroke(Gray4::WHITE, 1);

    if opts.screen.value == opt::Screen::Adsr {

        for (n_voice, voice) in voices.iter().enumerate() {
            draw_voice(d, (32*(n_voice % 4)) as i32, 32*(n_voice/4) as u32,
                       n_voice as u32, voice)?;
        }

    }

    if opts.screen.value == opt::Screen::Scope {
        let mut s: String<16> = String::new();
        ufmt::uwrite!(&mut s, "SCOPE").ok();
        draw_title_box(d, &s, Point::new(0, 0), Size::new(126, 64))?;

        let yn = 3;

        let stroke_grid = PrimitiveStyleBuilder::new()
            .stroke_color(Gray4::new(1))
            .stroke_width(1)
            .build();

        // Draw grainsz window
        Rectangle::new(Point::new(1, 10), Size::new(opts.scope.grain_sz.value/8, 53))
            .into_styled(stroke_grid)
            .draw(d)?;

        Line::new(Point::new(1, 32+yn),
                  Point::new(124, 32+yn))
                  .into_styled(stroke_grid)
                  .draw(d)?;

        Line::new(Point::new(64, 10),
                  Point::new(64, 62))
                  .into_styled(stroke_grid)
                  .draw(d)?;

        let trig_mv_px = opts.scope.trig_lvl.value >> 8;
        let trig_sns_px = opts.scope.trig_sns.value >> 8;

        Line::new(Point::new(1, 32+yn-trig_mv_px),
                  Point::new(124, 32+yn-trig_mv_px))
                  .into_styled(stroke_grid)
                  .draw(d)?;

        Line::new(Point::new(1, 32+yn-trig_sns_px-trig_mv_px),
                  Point::new(4, 32+yn-trig_sns_px-trig_mv_px))
                  .into_styled(stroke_grid)
                  .draw(d)?;
        Line::new(Point::new(1, 32+yn+trig_sns_px-trig_mv_px),
                  Point::new(4, 32+yn+trig_sns_px-trig_mv_px))
                  .into_styled(stroke_grid)
                  .draw(d)?;

        let mut points: [Point; 124] = [Point::new(0, 0); 124];
        for (n, point) in points.iter_mut().enumerate() {
            if n < scope_samples.len() {
                point.x = 1 + n as i32;
                point.y = 32+yn - (scope_samples[n] >> 10) as i32;
            }
        }
        Polyline::new(&points)
            .into_styled(thin_stroke)
            .draw(d)?;

    }

    if opts.screen.value == opt::Screen::Touch {
        let n_width = 8;
        for (n_touch, touch) in touch.iter().enumerate() {
            let mut s: String<16> = String::new();
            ufmt::uwrite!(&mut s, "{}", n_touch).ok();

            let stroke_gain = PrimitiveStyleBuilder::new()
                .stroke_color(Gray4::new(((*touch as u32 * 15u32) / 256u32) as u8))
                .stroke_width(1)
                .build();

            let px = 16 * (n_touch % n_width);
            let py = 16 * (n_touch / n_width);

            draw_title_box(d, &s, Point::new(px as i32, py as i32), Size::new(15, 16))?;
            Rectangle::new(Point::new(px as i32+1, py as i32+10), Size::new(13, 5))
                .into_styled(stroke_gain)
                .draw(d)?;
        }
    }

    if opts.screen.value == opt::Screen::Fluid {
        Rectangle::new(Point::new(0, 0), Size::new(128, 64))
            .into_styled(thin_stroke)
            .draw(d)?;

        let n_width = 8;
        for (n_touch, touch) in touch.iter().enumerate() {
            let px = 8 + 16 * (n_touch % n_width) as i32;
            let py = 8 + 16 * (n_touch / n_width) as i32;
            Line::new(Point::new(px-1, py),
                      Point::new(px+1, py))
                      .into_styled(thin_stroke)
                      .draw(d)?;
            Line::new(Point::new(px, py-1),
                      Point::new(px, py+1))
                      .into_styled(thin_stroke)
                      .draw(d)?;
        }

    }

    draw_options(d, &opts)?;

    let mut s: String<16> = String::new();
    ufmt::uwrite!(&mut s, "STATS").ok();
    draw_title_box(d, &s, Point::new(128, 0), Size::new(62, 64))?;

    draw_ms(d, "main", trace_main_len_us,
            Point::new(128+5, 17), font_small_white)?;
    draw_ms(d, "irq0", irq0_len_us,
            Point::new(128+5, 24), font_small_white)?;


    Ok(())
}

pub fn draw_boot_splash<D>(d: &mut D, state_string: &'static str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Gray4>,
{
    let mut s: String<64> = String::new();
    ufmt::uwrite!(&mut s, "PolyVec // Bootloader\n\napfelaudio.com").ok();
    let character_style_h = MonoTextStyle::new(&FONT_5X7, Gray4::WHITE);
    Text::with_alignment(
        &s,
        Point::new(128, 25),
        character_style_h,
        Alignment::Center,
    )
    .draw(d)?;

    s.clear();
    ufmt::uwrite!(&mut s, "{}", state_string).ok();
    let character_style_s = MonoTextStyle::new(&FONT_4X6, Gray4::WHITE);
    Text::with_alignment(
        &s,
        Point::new(128, 61),
        character_style_s,
        Alignment::Center,
    )
    .draw(d)?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use image::{ImageBuffer, RgbImage, Rgb};
    use image::imageops::{rotate90, resize, FilterType};
    use midi_types::MidiMessage;

    use crate::opt::Screen;

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
                if let Ok((x @ 0..=255, y @ 0..=63)) = coord.try_into() {
                    *self.img.get_pixel_mut(x, y) = Rgb([
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
        let mut opts = opt::Options::new();
        let mut voice_manager = VoiceManager::new();
        let scope_samples = [0i16; 128];
        let mut touch = [0u8; 32];
        touch[1] = 128;
        touch[4] = 255;

        voice_manager.event(MidiMessage::NoteOn(0.into(), 70.into(), 64.into()), 0u32);
        voice_manager.tick(10u32, &opts);
        voice_manager.tick(50u32, &opts);
        voice_manager.tick(200u32, &opts);

        let screens = [
            (Screen::Adsr, "adsr.png"),
            (Screen::Scope, "scope.png"),
            (Screen::Touch, "touch.png"),
            (Screen::Fluid, "fluid.png"),
        ];

        for (screen, filename) in screens {
            opts.screen.value = screen;
            disp.img = ImageBuffer::new(256, 64);
            draw_main(&mut disp, opts.clone(), voice_manager.voices.clone(), &scope_samples, &touch, 1, 2).unwrap();
            let rz = resize(&disp.img, 256*4, 64*4, FilterType::Nearest);
            rz.save(filename).unwrap();
        }
    }

    #[test]
    fn splash() {
        let mut disp = FakeDisplay {
            img: ImageBuffer::new(256, 64)
        };
        draw_boot_splash(&mut disp, "booting ...").unwrap();
        let rz = resize(&disp.img, 256*4, 64*4, FilterType::Nearest);
        rz.save("boot_splash.png").unwrap();
    }
}
