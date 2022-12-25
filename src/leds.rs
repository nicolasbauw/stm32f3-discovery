//! Provides access to User LEDs LD3-LD10
use stm32f3xx_hal::gpio::gpioe;
use stm32f3xx_hal::gpio::{Output, PushPull};

use switch_hal::{ActiveHigh, IntoSwitch, OutputSwitch, Switch};

use core::slice::Iter;
use core::iter::FusedIterator;

/// LED compass direction as noted on the board
#[derive(Clone, Copy, Eq, PartialEq)]
pub enum Direction
{
    North,
    NorthEast,
    East,
    SouthEast,
    South,
    SouthWest,
    West,
    NorthWest,
}

impl Direction {
    /// Provides an iterator starting with North
    /// and moving clockwise around the compass
    /// e.g. N -> NE -> E, etc.
    pub fn iter() -> Iter<'static, Direction> {
        static DIRECTIONS: [Direction; 8] = [
            Direction::North,
            Direction::NorthEast,
            Direction::East,
            Direction::SouthEast,
            Direction::South,
            Direction::SouthWest,
            Direction::West,
            Direction::NorthWest
        ];
        DIRECTIONS.iter()
    }
}

type Led = Switch<gpioe::PEx<Output<PushPull>>, ActiveHigh>;

pub struct Leds {
    /// North
    pub ld3_n: Led,
    /// NorthWest
    pub ld4_nw: Led,
    /// NorthEast
    pub ld5_ne: Led,
    /// West
    pub ld6_w: Led,
    /// East
    pub ld7_e: Led,
    /// SouthWest
    pub ld8_sw: Led,
    /// SouthEast
    pub ld9_se: Led,
    /// South
    pub ld10_s: Led,
}

impl Leds {
    /// Initializes the user LEDs to OFF
    pub fn new<PE8Mode, PE9Mode, PE10Mode, PE11Mode, PE12Mode, PE13Mode, PE14Mode, PE15Mode>(
        pe8: gpioe::PE8<PE8Mode>,
        pe9: gpioe::PE9<PE9Mode>,
        pe10: gpioe::PE10<PE10Mode>,
        pe11: gpioe::PE11<PE11Mode>,
        pe12: gpioe::PE12<PE12Mode>,
        pe13: gpioe::PE13<PE13Mode>,
        pe14: gpioe::PE14<PE14Mode>,
        pe15: gpioe::PE15<PE15Mode>,
        moder: &mut gpioe::MODER,
        otyper: &mut gpioe::OTYPER,
    ) -> Self {
        let mut leds = Leds {
            ld3_n: pe9
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
            ld4_nw: pe8
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
            ld5_ne: pe10
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
            ld6_w: pe15
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
            ld7_e: pe11
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
            ld8_sw: pe14
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
            ld9_se: pe12
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
            ld10_s: pe13
                .into_push_pull_output(moder, otyper)
                .downgrade()
                .into_active_high_switch(),
        };

        for led in &mut leds {
            led.off().ok();
        }

        leds
    }

    /// Mutably borrow a LED by the given direction (as noted on the board)
    /// 
    /// # Example
    /// 
    /// ```
    /// let southLed = leds.for_direction(Direction::South);
    /// southLed.on().ok();
    /// ```
    pub fn for_direction(&mut self, direction: Direction) -> &mut Led {
        match direction {
            Direction::North => &mut self.ld3_n,
            Direction::NorthEast => &mut self.ld5_ne,
            Direction::East => &mut self.ld7_e,
            Direction::SouthEast => &mut self.ld9_se,
            Direction::South => &mut self.ld10_s,
            Direction::SouthWest => &mut self.ld8_sw,
            Direction::West => &mut self.ld6_w,
            Direction::NorthWest => &mut self.ld4_nw,
        }
    }

    /// Provides a mutable iterator for iterating over the on board leds.
    /// Starts at ld3 (N) and moves clockwise.  
    /// Stops once it has iterated through all 8 leds.
    /// 
    /// # Examples
    /// 
    /// Iterate over the leds clockwise
    /// 
    /// ```
    /// let ms_delay = 50u16;
    /// for led in &mut leds {
    ///     led.on().ok();
    ///     delay.delay_ms(ms_delay);
    ///     led.off().ok();
    ///     delay.delay_ms(ms_delay);
    /// }
    /// ```
    /// 
    /// Iterate over the leds counter clockwise
    /// 
    /// ```
    /// let ms_delay = 50u16;
    /// for led in leds.iter_mut().rev() {
    ///     led.on().ok();
    ///     delay.delay_ms(ms_delay);
    ///     led.off().ok();
    ///     delay.delay_ms(ms_delay);
    /// }
    /// ```
    pub fn iter_mut(&mut self) -> LedsMutIterator {
        LedsMutIterator::new(self)
    }

    /// Consumes the `Leds` struct and returns an array,
    /// where index 0 is N and each incrementing index.  
    /// Rotates clockwise around the compass.
    /// 
    /// # Warning
    /// 
    /// This function is maintained solely for some level of compatibility with the old F3 crate.
    /// 
    /// [`Self::iter_mut()`] should be prefered.
    /// Testing suggests that using [`Self::iter_mut()`] results in an ~800 byte
    /// reduction in final binary size.
    pub fn into_array(self) -> [Led; 8] {
        [
            self.ld3_n,  //N
            self.ld5_ne,  //NE
            self.ld7_e,  //E
            self.ld9_se,  //SE
            self.ld10_s, //S
            self.ld8_sw,  //SW
            self.ld6_w,  //W
            self.ld4_nw,  //NW
        ]
    }
}

impl<'a> IntoIterator for &'a mut Leds {
    type Item = &'a mut Led;
    type IntoIter = LedsMutIterator<'a>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter_mut()
    }
}

const ITERATOR_SIZE: usize = 8;

pub struct LedsMutIterator<'a> {
    index: usize,
    index_back: usize,
    leds: &'a mut Leds
}

impl<'a> LedsMutIterator<'a> {
    fn new(leds: &'a mut Leds) -> Self {
        LedsMutIterator { index: 0, index_back: ITERATOR_SIZE, leds }
    }

    fn len(&self) -> usize {
        self.index_back - self.index
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let length = self.len();
        (length, Some(length))
    }
}

impl<'a> Iterator for LedsMutIterator<'a> {
    type Item = &'a mut Led;
    fn next(&mut self) -> Option<Self::Item> {
        if self.len() == 0 {
            None
        } else {
            let current = unsafe {
                //Safety: Each branch is only executed once,
                // and only if there are elements left to be returned, 
                // so we can not possibly alias a mutable reference.
                // This depends on DoubleEndedIterator and ExactSizedIterator being implemented correctly.
                // If len() does not return the correct number of remaining elements, 
                // this becomes unsound.
                    match self.index {
                        0 => Some(&mut *(&mut self.leds.ld3_n as *mut _)),  //N
                        1 => Some(&mut *(&mut self.leds.ld5_ne as *mut _)),  //NE
                        2 => Some(&mut *(&mut self.leds.ld7_e as *mut _)),  //E
                        3 => Some(&mut *(&mut self.leds.ld9_se as *mut _)),  //SE
                        4 => Some(&mut *(&mut self.leds.ld10_s as *mut _)), //S
                        5 => Some(&mut *(&mut self.leds.ld8_sw as *mut _)),  //SW
                        6 => Some(&mut *(&mut self.leds.ld6_w as *mut _)),  //W
                        7 => Some(&mut *(&mut self.leds.ld4_nw as *mut _)),  //NW
                        _ => None
                }
            };
            self.index += 1;
            current
        }
    }

    // Because we implement ExactSizedIterator, we need to ensure size_hint returns the right length
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.size_hint()
    }
}

impl<'a> DoubleEndedIterator for LedsMutIterator<'a> {
    fn next_back(&mut self) -> Option<Self::Item> { 
        if self.len() == 0 {
            None
        } else {
            let current = unsafe {
                //Safety: Each branch is only executed once,
                // and only if there are elements left to be returned, 
                // so we can not possibly alias a mutable reference.
                // This depends on Iterator and ExactSizedIterator being implemented correctly.
                // If len() does not return the correct number of remaining elements, 
                // this becomes unsound.
                    match self.index_back {
                        // Because we're going backwards and index_back is a usize,
                        // We use a one based index so we don't go negative
                        0 => None, //done
                        1 => Some(&mut *(&mut self.leds.ld3_n as *mut _)),  //N
                        2 => Some(&mut *(&mut self.leds.ld5_ne as *mut _)),  //NE
                        3 => Some(&mut *(&mut self.leds.ld7_e as *mut _)),  //E
                        4 => Some(&mut *(&mut self.leds.ld9_se as *mut _)),  //SE
                        5 => Some(&mut *(&mut self.leds.ld10_s as *mut _)), //S
                        6 => Some(&mut *(&mut self.leds.ld8_sw as *mut _)),  //SW
                        7 => Some(&mut *(&mut self.leds.ld6_w as *mut _)),  //W
                        8 => Some(&mut *(&mut self.leds.ld4_nw as *mut _)),  //NW
                        _ => None //can't happen
                }
            };
            self.index_back -= 1;
            current
        }
    }
}

impl<'a> ExactSizeIterator for LedsMutIterator<'a> {
    fn len(&self) -> usize {
        self.len()
    }
}

///Marker trait that indicates LedsMutIterator never starts returning Some after returning None
impl<'a> FusedIterator for LedsMutIterator<'a> {}