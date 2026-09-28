library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_2mat_conv is
  port(
    clk40  : in  std_logic;
    rx     : in  std_logic;

    tx     : out std_logic;   -- <<< ADDED: UART TX to CoolTerm

    n_out  : out std_logic_vector(3 downto 0);
    m_out  : out std_logic_vector(3 downto 0);
    outN   : out std_logic_vector(3 downto 0);      -- out size = n-m+1

    A      : out std_logic_vector(9 downto 0)(79 downto 0); -- 10 rows × 10 bytes
    B      : out std_logic_vector(9 downto 0)(79 downto 0);

    done   : out std_logic;                          -- goes high after conv is computed
    C_flat : out std_logic_vector(3199 downto 0)     -- 10*10 results packed, each 32-bit
  );
end;

architecture rtl of uart_2mat_conv is
  constant DIV : integer := 174; -- 40e6 / 115200

  -- baud tick (RX)
  signal cnt  : integer range 0 to DIV-1 := 0;
  signal tick : std_logic := '0';

  -- UART RX (very simple)
  type rx_state_t is (IDLE, DATA, STOP);
  signal rx_state : rx_state_t := IDLE;
  signal bit_i    : integer range 0 to 7 := 0;
  signal rx_byte  : std_logic_vector(7 downto 0) := (others=>'0');
  signal rx_done  : std_logic := '0';

  -- receive stages
  type stage_t is (GET_N, GET_M, READ_A, READ_B, FINISH);
  signal stage : stage_t := GET_N;

  signal n, m    : integer range 0 to 10 := 0;
  signal totalA  : integer range 0 to 100 := 0;
  signal totalB  : integer range 0 to 100 := 0;
  signal cntA    : integer range 0 to 100 := 0;
  signal cntB    : integer range 0 to 100 := 0;

  signal bufA : std_logic_vector(9 downto 0)(79 downto 0) := (others => (others => '0'));
  signal bufB : std_logic_vector(9 downto 0)(79 downto 0) := (others => (others => '0'));

  -- convolution output
  signal Cbuf      : std_logic_vector(3199 downto 0) := (others => '0');
  signal conv_done : std_logic := '0';
  signal outN_sig  : std_logic_vector(3 downto 0) := (others => '0');

  ------------------------------------------------------------------------
  -- ADDED: UART TX + sender that sends all 100 slots of Cbuf to CoolTerm
  ------------------------------------------------------------------------

  -- independent TX tick (so we don't touch your RX tick logic)
  signal cnt_tx  : integer range 0 to DIV-1 := 0;
  signal tick_tx : std_logic := '0';

  -- UART TX engine (8N1)
  signal tx_line  : std_logic := '1';
  signal tx_busy  : std_logic := '0';
  signal tx_shift : std_logic_vector(9 downto 0) := (others => '1');
  signal tx_bit_i : integer range 0 to 9 := 0;

  signal tx_start : std_logic := '0';
  signal tx_byte  : std_logic_vector(7 downto 0) := (others => '0');

  -- start once after conv_done becomes 1 (and never restart)
  signal send_started : std_logic := '0';
  signal send_done    : std_logic := '0';

  type txsend_state_t is (TX_IDLE, TX_LOAD_WORD, TX_SEND_DIGIT, TX_SEND_CR, TX_SEND_LF, TX_NEXT_WORD);
  signal txs_state : txsend_state_t := TX_IDLE;

  signal word_idx : integer range 0 to 99 := 0;

  -- decimal digits buffer: up to 10 digits for 32-bit
  signal dig_buf : std_logic_vector(79 downto 0) := (others => '0'); -- 10 bytes
  signal dig_len : integer range 1 to 10 := 1;
  signal dig_pos : integer range 0 to 9 := 0;

begin
  A      <= bufA;
  B      <= bufB;
  C_flat <= Cbuf;

  n_out <= std_logic_vector(to_unsigned(n, 4));
  m_out <= std_logic_vector(to_unsigned(m, 4));
  outN  <= outN_sig;

  done  <= conv_done;

  -- TX output
  tx <= tx_line;

  --------------------------------------------------------------------------
  -- YOUR ORIGINAL PROCESS (UNCHANGED)
  --------------------------------------------------------------------------
  process(clk40)
    variable tmp     : integer;
    variable row_i   : integer;
    variable col_i   : integer;

    variable r, c, kr, kc : integer;
    variable out_size     : integer;
    variable sum          : unsigned(31 downto 0);
    variable a_val, b_val : unsigned(7 downto 0);
    variable base         : integer;
  begin
    if rising_edge(clk40) then
      ----------------------------------------------------------------
      -- 1x baud tick
      ----------------------------------------------------------------
      tick <= '0';
      if cnt = DIV-1 then
        cnt  <= 0;
        tick <= '1';
      else
        cnt <= cnt + 1;
      end if;

      ----------------------------------------------------------------
      -- UART RX: IDLE -> DATA -> STOP
      ----------------------------------------------------------------
      rx_done <= '0';

      if tick='1' then
        case rx_state is
          when IDLE =>
            if rx='0' then
              bit_i <= 0;
              rx_state <= DATA;
            end if;

          when DATA =>
            rx_byte(bit_i) <= rx; -- LSB first
            if bit_i = 7 then
              rx_state <= STOP;
            else
              bit_i <= bit_i + 1;
            end if;

          when STOP =>
            if rx='1' then
              rx_done <= '1';
            end if;
            rx_state <= IDLE;
        end case;
      end if;

      ----------------------------------------------------------------
      -- Receive 2 matrices: n, m, A, B
      ----------------------------------------------------------------
      if rx_done='1' and stage /= FINISH then
        case stage is
          when GET_N =>
            tmp := to_integer(unsigned(rx_byte));
            if tmp >= 1 and tmp <= 10 then
              n      <= tmp;
              totalA <= tmp * tmp;
              cntA   <= 0;
              bufA   <= (others => (others => '0'));
              stage  <= GET_M;
              conv_done <= '0';
              Cbuf <= (others => '0');
              outN_sig <= (others => '0');
            end if;

          when GET_M =>
            tmp := to_integer(unsigned(rx_byte));
            if tmp >= 1 and tmp <= 10 then
              m      <= tmp;
              totalB <= tmp * tmp;
              cntB   <= 0;
              bufB   <= (others => (others => '0'));
              stage  <= READ_A;
            end if;

          when READ_A =>
            row_i := cntA / n;
            col_i := cntA mod n;
            bufA(row_i)(col_i*8+7 downto col_i*8) <= rx_byte;
            cntA <= cntA + 1;
            if (cntA + 1) = totalA then
              stage <= READ_B;
            end if;

          when READ_B =>
            row_i := cntB / m;
            col_i := cntB mod m;
            bufB(row_i)(col_i*8+7 downto col_i*8) <= rx_byte;
            cntB <= cntB + 1;
            if (cntB + 1) = totalB then
              stage <= FINISH;
            end if;

          when FINISH =>
            null;
        end case;
      end if;

      ----------------------------------------------------------------
      -- Convolution once after receive finished (valid, no padding)
      -- C(r,c) = sum_{kr,kc} A(r+kr,c+kc) * B(kr,kc)
      ----------------------------------------------------------------
      if (stage = FINISH) and (conv_done = '0') then
        conv_done <= '1';
        Cbuf <= (others => '0');

        out_size := n - m + 1; -- no error handling as you requested
        outN_sig <= std_logic_vector(to_unsigned(out_size, 4));

        for r in 0 to out_size-1 loop
          for c in 0 to out_size-1 loop
            sum := (others => '0');

            for kr in 0 to m-1 loop
              for kc in 0 to m-1 loop
                a_val := unsigned(bufA(r+kr)((c+kc)*8+7 downto (c+kc)*8));
                b_val := unsigned(bufB(kr)(kc*8+7 downto kc*8));
                sum := sum + resize(a_val * b_val, 32);
              end loop;
            end loop;

            base := (r*10 + c) * 32; -- pack into 10x10 slots
            Cbuf(base+31 downto base) <= std_logic_vector(sum);
          end loop;
        end loop;
      end if;

    end if;
  end process;

  --------------------------------------------------------------------------
  -- ADDED PROCESS #1: TX baud tick + UART TX engine (8N1 @ 115200)
  --------------------------------------------------------------------------
  process(clk40)
  begin
    if rising_edge(clk40) then
      -- TX tick
      tick_tx <= '0';
      if cnt_tx = DIV-1 then
        cnt_tx  <= 0;
        tick_tx <= '1';
      else
        cnt_tx <= cnt_tx + 1;
      end if;

      -- default: clear tx_start after 1 cycle
      if tx_start = '1' then
        tx_start <= '0';
      end if;

      if tick_tx = '1' then
        if tx_busy = '0' then
          tx_line <= '1'; -- idle
          if tx_start = '1' then
            -- frame: start(0) + data(LSB first) + stop(1)
            tx_shift <= '1' & tx_byte & '0';
            tx_busy  <= '1';
            tx_bit_i <= 0;
          end if;
        else
          tx_line <= tx_shift(tx_bit_i);
          if tx_bit_i = 9 then
            tx_busy <= '0';
            tx_line <= '1';
          else
            tx_bit_i <= tx_bit_i + 1;
          end if;
        end if;
      end if;

    end if;
  end process;

  --------------------------------------------------------------------------
  -- ADDED PROCESS #2: When conv_done becomes 1, send all 100 slots of Cbuf
  -- Format in CoolTerm: DECIMAL ASCII + CRLF per number
  --------------------------------------------------------------------------
  process(clk40)
    variable v      : integer;
    variable tmp    : integer;
    variable digits : integer;
    variable i      : integer;
    variable b      : std_logic_vector(7 downto 0);
    variable word32 : std_logic_vector(31 downto 0);
  begin
    if rising_edge(clk40) then

      -- latch start forever once conv_done goes high (even if your code later clears it)
      if conv_done = '1' then
        send_started <= '1';
      end if;

      case txs_state is
        when TX_IDLE =>
          if (send_started = '1') and (send_done = '0') then
            word_idx  <= 0;
            txs_state <= TX_LOAD_WORD;
          end if;

        when TX_LOAD_WORD =>
          if tx_busy = '0' then
            word32 := Cbuf(word_idx*32 + 31 downto word_idx*32);
            v := to_integer(unsigned(word32));

            -- make decimal ASCII digits into dig_buf (no leading zeros)
            if v = 0 then
              dig_len <= 1;
              dig_buf(7 downto 0) <= x"30"; -- '0'
            else
              tmp := v;
              digits := 0;
              while tmp > 0 loop
                tmp := tmp / 10;
                digits := digits + 1;
              end loop;
              if digits < 1 then digits := 1; end if;
              if digits > 10 then digits := 10; end if;

              dig_len <= digits;

              -- clear buffer
              for i in 0 to 9 loop
                dig_buf(i*8+7 downto i*8) <= x"30";
              end loop;

              -- fill from most significant to least
              tmp := v;
              for i in 0 to digits-1 loop
                b := std_logic_vector(to_unsigned((tmp mod 10) + 48, 8)); -- +48 => ASCII
                dig_buf((digits-1-i)*8+7 downto (digits-1-i)*8) <= b;
                tmp := tmp / 10;
              end loop;
            end if;

            dig_pos   <= 0;
            txs_state <= TX_SEND_DIGIT;
          end if;

        when TX_SEND_DIGIT =>
          if tx_busy = '0' then
            tx_byte  <= dig_buf(dig_pos*8+7 downto dig_pos*8);
            tx_start <= '1';

            if dig_pos = dig_len-1 then
              txs_state <= TX_SEND_CR;
            else
              dig_pos <= dig_pos + 1;
            end if;
          end if;

        when TX_SEND_CR =>
          if tx_busy = '0' then
            tx_byte  <= x"0D"; -- CR
            tx_start <= '1';
            txs_state <= TX_SEND_LF;
          end if;

        when TX_SEND_LF =>
          if tx_busy = '0' then
            tx_byte  <= x"0A"; -- LF
            tx_start <= '1';
            txs_state <= TX_NEXT_WORD;
          end if;

        when TX_NEXT_WORD =>
          if tx_busy = '0' then
            if word_idx = 99 then
              send_done <= '1';     -- never send again
              txs_state <= TX_IDLE;
            else
              word_idx  <= word_idx + 1;
              txs_state <= TX_LOAD_WORD;
            end if;
          end if;

      end case;

    end if;
  end process;

end rtl;
