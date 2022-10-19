/** @brief Stop timer, prescaler 000 --> STOP */
#define TIM2_stop()             TCCR2B &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
/** @brief Set overflow 16us, prescaler 001 --> 1 */
#define TIM2_overflow_16us()    TCCR2B &= ~((1<<CS22) | (1<<CS21)); TCCR2B |= (1<<CS20);
/** @brief Set overflow 128us, prescaler 010 --> 8 */
#define TIM2_overflow_128us()   TCCR2B &= ~((1<<CS22) | (1<<CS20)); TCCR2B |= (1<<CS21);
/** @brief Set overflow 512us, prescaler 011 --> 32*/
#define TIM2_overflow_512us()   TCCR2B &= ~(1<<CS22); TCCR2B |= (1<<CS21) | (1<<CS20);
/** @brief Set overflow 1ms, prescaler 100 --> 64 */
#define TIM2_overflow_1ms()      TCCR2B &= ~(1<<CS21) | (1<<CS20); TCCR2B |= (1<<CS22);
/** @brief Set overflow 2ms, prescaler 101 --> 128 */
#define TIM2_overflow_2ms()      TCCR2B &= ~(1<<CS21); TCCR2B |= (1<<CS22) | (1<<CS20);
/** @brief Set overflow 4ms, prescaler 110 --> 256 */
#define TIM2_overflow_4ms()      TCCR2B &= ~(1<<CS20); TCCR2B |= (1<<CS22) | (1<<CS21);
/** @brief Set overflow 16ms, prescaler // 111 --> 1024 */
#define TIM2_overflow_16ms()     TCCR2B |= ((1<<CS20) | (1<<CS21) | (1<<CS22));

/** @brief Enable overflow interrupt, 1 --> enable */
#define TIM2_overflow_interrupt_enable()  TIMSK2 |= (1<<TOIE2);
/** @brief Disable overflow interrupt, 0 --> disable */
#define TIM2_overflow_interrupt_disable() TIMSK2 &= ~(1<<TOIE2);