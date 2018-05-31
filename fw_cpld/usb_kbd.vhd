library ieee;
use ieee.std_logic_1164.all;

entity usb_kbd is

	port
	(
		clk		 : in	std_logic;
		
		ziorqn	: in	std_logic;
		zmreqn	: in	std_logic;
		zrdn		: in	std_logic;
		zwrn		: in	std_logic;
		zm1n		: in	std_logic;
		zdata		: inout	std_logic_vector( 7 downto 0);
		zaddr	 	: in	std_logic_vector(15 downto 0);
		zdosn		: in	std_logic;
		tin		: in	std_logic;
		dasel		: in	std_logic;
		f14		: in	std_logic;
		csrn		: in	std_logic;
		iintu		: in	std_logic;
		
		zrstn		: out	std_logic;
		iorqgen	: out	std_logic;
		iodosn	: out	std_logic;
		int0n		: out	std_logic;
		turbo		: out	std_logic;
		rdrn		: out	std_logic;
		blk		: out	std_logic;
		ointu		: out	std_logic;
		extan		: out	std_logic;
		ula		: out	std_logic;
		da			: inout	std_logic_vector( 7 downto 0);
		
		vector 	: out	std_logic_vector(7 downto 0);
		sintn		: out	std_logic
	);

end entity;

architecture rtl of usb_kbd is

signal int_to_stm	: std_logic;
signal int_vector	: std_logic_vector(7 downto 0);

signal zaddr_full	: std_logic_vector(16 downto 0);

begin

zaddr_full(15 downto 0) <= zaddr;

process (clk)
begin
	if (rising_edge(clk)) then
		if (ziorqn = '1') then
			int_vector <= (others => '0');
			int_to_stm <= '1';
		else
			if (zaddr_full = "00000000011011111") then	-- version ZXMC
				int_vector <= "00000001";
				int_to_stm <= '0';
			elsif ((zaddr_full(16) = '1') and (zaddr_full(7 downto 0) = "00011111")) then --Kjoy
				int_vector <= "00000010";
				int_to_stm <= '0';
			elsif (zaddr_full(7 downto 0) = "01010111") then --SD data
				int_vector <= "00000011";
				int_to_stm <= '0';
			elsif (zaddr_full(7 downto 0) = "01110111") then --SD command
				int_vector <= "00000100";
				int_to_stm <= '0';
			elsif ((zaddr_full(16) = '1') and (zaddr_full(7 downto 0) = "01111111")) then --Fjoy
				int_vector <= "00000101";
				int_to_stm <= '0';
			elsif ((zaddr_full(16) = '1') and (zaddr_full(7 downto 0) = "11011111")) then --Kjoy
				int_vector <= "00000110";
				int_to_stm <= '0';
			else
				int_vector <= (others => '0');
				int_to_stm <= '1';
			end if;
		end if;
	end if;
end process;

-- to outputs
sintn <= not int_to_stm;
iorqgen <= not int_to_stm;
vector <= int_vector;

end rtl;
