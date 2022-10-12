#include <stdio.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "driver/i2c.h"
#include "esp_console.h"
#include "esp_log.h"

#define TAG "cmd_lcd1602"

#define LCD_NUM_ROWS 2
#define LCD_NUM_COLUMNS 32
#define LCD_NUM_VISIBLE_COLUMNS 16

// Special characters for ROM Code A00

// Use the second set (0bxxxx1xxx) to avoid placing the null character within a string
#define I2C_LCD1602_CHARACTER_CUSTOM_0 0b00001000 ///< User-defined custom symbol in index 0
#define I2C_LCD1602_CHARACTER_CUSTOM_1 0b00001001 ///< User-defined custom symbol in index 1
#define I2C_LCD1602_CHARACTER_CUSTOM_2 0b00001010 ///< User-defined custom symbol in index 2
#define I2C_LCD1602_CHARACTER_CUSTOM_3 0b00001011 ///< User-defined custom symbol in index 3
#define I2C_LCD1602_CHARACTER_CUSTOM_4 0b00001100 ///< User-defined custom symbol in index 4
#define I2C_LCD1602_CHARACTER_CUSTOM_5 0b00001101 ///< User-defined custom symbol in index 5
#define I2C_LCD1602_CHARACTER_CUSTOM_6 0b00001110 ///< User-defined custom symbol in index 6
#define I2C_LCD1602_CHARACTER_CUSTOM_7 0b00001111 ///< User-defined custom symbol in index 7

#define I2C_LCD1602_CHARACTER_ALPHA 0b11100000       ///< Lower-case alpha symbol
#define I2C_LCD1602_CHARACTER_BETA 0b11100010        ///< Lower-case beta symbol
#define I2C_LCD1602_CHARACTER_THETA 0b11110010       ///< Lower-case theta symbol
#define I2C_LCD1602_CHARACTER_PI 0b11110111          ///< Lower-case pi symbol
#define I2C_LCD1602_CHARACTER_OMEGA 0b11110100       ///< Upper-case omega symbol
#define I2C_LCD1602_CHARACTER_SIGMA 0b11110110       ///< Upper-case sigma symbol
#define I2C_LCD1602_CHARACTER_INFINITY 0b11110011    ///< Infinity symbol
#define I2C_LCD1602_CHARACTER_DEGREE 0b11011111      ///< Degree symbol
#define I2C_LCD1602_CHARACTER_ARROW_RIGHT 0b01111110 ///< Arrow pointing right symbol
#define I2C_LCD1602_CHARACTER_ARROW_LEFT 0b01111111  ///< Arrow pointing left symbol
#define I2C_LCD1602_CHARACTER_SQUARE 0b11011011      ///< Square outline symbol
#define I2C_LCD1602_CHARACTER_DOT 0b10100101         ///< Centred dot symbol
#define I2C_LCD1602_CHARACTER_DIVIDE 0b11111101      ///< Division sign symbol
#define I2C_LCD1602_CHARACTER_BLOCK 0b11111111       ///< 5x8 filled block

// Delays (microseconds)
#define DELAY_POWER_ON 50000 // wait at least 40us after VCC rises to 2.7V
#define DELAY_INIT_1 4500    // wait at least 4.1ms (fig 24, page 46)
#define DELAY_INIT_2 4500    // wait at least 4.1ms (fig 24, page 46)
#define DELAY_INIT_3 120     // wait at least 100us (fig 24, page 46)

#define DELAY_CLEAR_DISPLAY 2000
#define DELAY_RETURN_HOME 2000

#define DELAY_ENABLE_PULSE_WIDTH 1   // enable pulse must be at least 450ns wide
#define DELAY_ENABLE_PULSE_SETTLE 50 // command requires > 37us to settle (table 6 in datasheet)

// Commands
#define COMMAND_CLEAR_DISPLAY 0x01
#define COMMAND_RETURN_HOME 0x02
#define COMMAND_ENTRY_MODE_SET 0x04
#define COMMAND_DISPLAY_CONTROL 0x08
#define COMMAND_SHIFT 0x10
#define COMMAND_FUNCTION_SET 0x20
#define COMMAND_SET_CGRAM_ADDR 0x40
#define COMMAND_SET_DDRAM_ADDR 0x80

// COMMAND_ENTRY_MODE_SET flags
#define FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT 0x02
#define FLAG_ENTRY_MODE_SET_ENTRY_DECREMENT 0x00
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON 0x01
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF 0x00

// COMMAND_DISPLAY_CONTROL flags
#define FLAG_DISPLAY_CONTROL_DISPLAY_ON 0x04
#define FLAG_DISPLAY_CONTROL_DISPLAY_OFF 0x00
#define FLAG_DISPLAY_CONTROL_CURSOR_ON 0x02
#define FLAG_DISPLAY_CONTROL_CURSOR_OFF 0x00
#define FLAG_DISPLAY_CONTROL_BLINK_ON 0x01
#define FLAG_DISPLAY_CONTROL_BLINK_OFF 0x00

// COMMAND_SHIFT flags
#define FLAG_SHIFT_MOVE_DISPLAY 0x08
#define FLAG_SHIFT_MOVE_CURSOR 0x00
#define FLAG_SHIFT_MOVE_LEFT 0x04
#define FLAG_SHIFT_MOVE_RIGHT 0x00

// COMMAND_FUNCTION_SET flags
#define FLAG_FUNCTION_SET_MODE_8BIT 0x10
#define FLAG_FUNCTION_SET_MODE_4BIT 0x00
#define FLAG_FUNCTION_SET_LINES_2 0x08
#define FLAG_FUNCTION_SET_LINES_1 0x00
#define FLAG_FUNCTION_SET_DOTS_5X10 0x04
#define FLAG_FUNCTION_SET_DOTS_5X8 0x00

// Control flags
#define FLAG_BACKLIGHT_ON 0b00001000  // backlight enabled (disabled if clear)
#define FLAG_BACKLIGHT_OFF 0b00000000 // backlight disabled
#define FLAG_ENABLE 0b00000100
#define FLAG_READ 0b00000010       // read (write if clear)
#define FLAG_WRITE 0b00000000      // write
#define FLAG_RS_DATA 0b00000001    // data (command if clear)
#define FLAG_RS_COMMAND 0b00000000 // command

#define I2C_LCD1602_ERROR_CHECK(x)                                          \
    do                                                                      \
    {                                                                       \
        esp_err_t rc = (x);                                                 \
        if (rc != ESP_OK)                                                   \
        {                                                                   \
            ESP_LOGW(TAG, "I2C error %d at %s:%d", rc, __FILE__, __LINE__); \
        }                                                                   \
    } while (0);

/**
 * @brief Enum of valid indexes for definitions of user-defined characters.
 */
typedef enum
{
    I2C_LCD1602_INDEX_CUSTOM_0 = 0, ///< Index of first user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_1,     ///< Index of second user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_2,     ///< Index of third user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_3,     ///< Index of fourth user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_4,     ///< Index of fifth user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_5,     ///< Index of sixth user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_6,     ///< Index of seventh user-defined custom symbol
    I2C_LCD1602_INDEX_CUSTOM_7,     ///< Index of eighth user-defined custom symbol
} i2c_lcd1602_custom_index_t;

/**
 * @brief Structure containing information related to the I2C-LCD1602 device.
 */
typedef struct
{
    bool init;                     ///< True if struct has been initialised, otherwise false
    i2c_port_t i2c_port;           ///< Pointer to associated I2C Port
    uint8_t backlight_flag;        ///< Non-zero if backlight is to be enabled, otherwise zero
    uint8_t num_rows;              ///< Number of configured columns
    uint8_t num_columns;           ///< Number of configured columns, including offscreen columns
    uint8_t num_visible_columns;   ///< Number of visible columns
    uint8_t display_control_flags; ///< Currently active display control flags
    uint8_t entry_mode_flags;      ///< Currently active entry mode flags
} i2c_lcd1602_info_t;

i2c_lcd1602_info_t *lcd_info;

static bool _is_init(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    bool ok = false;
    if (i2c_lcd1602_info != NULL)
    {
        if (i2c_lcd1602_info->init)
        {
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "i2c_lcd1602_info is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "i2c_lcd1602_info is NULL");
    }
    return ok;
}

// Set or clear the specified flag depending on condition
static uint8_t _set_or_clear(uint8_t flags, bool condition, uint8_t flag)
{
    if (condition)
    {
        flags |= flag;
    }
    else
    {
        flags &= ~flag;
    }
    return flags;
}

// send data to the I/O Expander
static esp_err_t _write_to_expander(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_LOGD(TAG, "_write_to_expander 0x%02x", data);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x27 << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, data | i2c_lcd1602_info->backlight_flag, 1);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_lcd1602_info->i2c_port, cmd, 50 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    return ret;
}

// IMPORTANT - for the display to stay "in sync" it is important that errors do not interrupt the
// 2 x nibble sequence.

// clock data from expander to LCD by causing a falling edge on Enable
static esp_err_t _strobe_enable(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t data)
{
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data | FLAG_ENABLE);
    ets_delay_us(DELAY_ENABLE_PULSE_WIDTH);
    esp_err_t err2 = _write_to_expander(i2c_lcd1602_info, data & ~FLAG_ENABLE);
    ets_delay_us(DELAY_ENABLE_PULSE_SETTLE);
    return err1 ? err1 : err2;
}

// send top nibble to the LCD controller
static esp_err_t _write_top_nibble(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t data)
{
    ESP_LOGD(TAG, "_write_top_nibble 0x%02x", data);
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data);
    esp_err_t err2 = _strobe_enable(i2c_lcd1602_info, data);
    return err1 ? err1 : err2;
}

// send command or data to controller
static esp_err_t _write(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t value, uint8_t register_select_flag)
{
    ESP_LOGD(TAG, "_write 0x%02x | 0x%02x", value, register_select_flag);
    esp_err_t err1 = _write_top_nibble(i2c_lcd1602_info, (value & 0xf0) | register_select_flag);
    esp_err_t err2 = _write_top_nibble(i2c_lcd1602_info, ((value & 0x0f) << 4) | register_select_flag);
    return err1 ? err1 : err2;
}

// send command to controller
static esp_err_t _write_command(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t command)
{
    ESP_LOGD(TAG, "_write_command 0x%02x", command);
    return _write(i2c_lcd1602_info, command, FLAG_RS_COMMAND);
}

// send data to controller
static esp_err_t _write_data(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t data)
{
    ESP_LOGD(TAG, "_write_data 0x%02x", data);
    return _write(i2c_lcd1602_info, data, FLAG_RS_DATA);
}

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 18,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = 19,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    return i2c_param_config(I2C_NUM_0, &conf);
}

static i2c_lcd1602_info_t *i2c_lcd1602_malloc(void)
{
    i2c_lcd1602_info_t *i2c_lcd1602_info = malloc(sizeof(*i2c_lcd1602_info));
    if (i2c_lcd1602_info != NULL)
    {
        memset(i2c_lcd1602_info, 0, sizeof(*i2c_lcd1602_info));
        ESP_LOGD(TAG, "malloc i2c_lcd1602_info_t %p", i2c_lcd1602_info);
    }
    else
    {
        ESP_LOGE(TAG, "malloc i2c_lcd1602_info_t failed");
    }
    return i2c_lcd1602_info;
}

static void i2c_lcd1602_free(i2c_lcd1602_info_t **i2c_lcd1602_info)
{
    if (i2c_lcd1602_info != NULL && (*i2c_lcd1602_info != NULL))
    {
        ESP_LOGD(TAG, "free i2c_lcd1602_info_t %p", *i2c_lcd1602_info);
        free(*i2c_lcd1602_info);
        *i2c_lcd1602_info = NULL;
    }
    else
    {
        ESP_LOGE(TAG, "free i2c_lcd1602_info_t failed");
    }
}

static esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        err = _write_command(i2c_lcd1602_info, COMMAND_CLEAR_DISPLAY);
        if (err == ESP_OK)
        {
            ets_delay_us(DELAY_CLEAR_DISPLAY);
        }
    }
    return err;
}

static esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        err = _write_command(i2c_lcd1602_info, COMMAND_RETURN_HOME);
        if (err == ESP_OK)
        {
            ets_delay_us(DELAY_RETURN_HOME);
        }
    }
    return err;
}

static esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t first_err = ESP_OK;
    esp_err_t last_err = ESP_FAIL;

    // put Expander into known state - Register Select and Read/Write both low
    if ((last_err = _write_to_expander(i2c_lcd1602_info, 0)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_to_expander 1 failed: %d", last_err);
    }

    ets_delay_us(1000);

    // select 4-bit mode on LCD controller - see datasheet page 46, figure 24.
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 1 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_1);

    // repeat
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 2 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_2);

    // repeat
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x03 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 3 failed: %d", last_err);
    }

    ets_delay_us(DELAY_INIT_3);

    // select 4-bit mode
    if ((last_err = _write_top_nibble(i2c_lcd1602_info, 0x02 << 4)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 4 failed: %d", last_err);
    }

    // now we can use the command()/write() functions
    if ((last_err = _write_command(i2c_lcd1602_info, COMMAND_FUNCTION_SET | FLAG_FUNCTION_SET_MODE_4BIT | FLAG_FUNCTION_SET_LINES_2 | FLAG_FUNCTION_SET_DOTS_5X8)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 1 failed: %d", last_err);
    }

    if ((last_err = _write_command(i2c_lcd1602_info, COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 2 failed: %d", last_err);
    }

    if ((last_err = i2c_lcd1602_clear(i2c_lcd1602_info)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_clear failed: %d", last_err);
    }

    if ((last_err = _write_command(i2c_lcd1602_info, COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 3 failed: %d", last_err);
    }

    if ((last_err = i2c_lcd1602_home(i2c_lcd1602_info)) != ESP_OK)
    {
        if (first_err == ESP_OK)
            first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_home failed: %d", last_err);
    }

    return first_err;
}

static esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t *i2c_lcd1602_info, i2c_port_t i2c_port,
                                  bool backlight, uint8_t num_rows, uint8_t num_columns, uint8_t num_visible_columns)
{
    esp_err_t err = ESP_FAIL;
    if (i2c_lcd1602_info != NULL)
    {
        i2c_lcd1602_info->i2c_port = i2c_port;
        i2c_lcd1602_info->backlight_flag = backlight ? FLAG_BACKLIGHT_ON : FLAG_BACKLIGHT_OFF;
        i2c_lcd1602_info->num_rows = num_rows;
        i2c_lcd1602_info->num_columns = num_columns;
        i2c_lcd1602_info->num_visible_columns = num_visible_columns;

        i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
        i2c_master_driver_initialize();

        // display on, no cursor, no blinking
        i2c_lcd1602_info->display_control_flags = FLAG_DISPLAY_CONTROL_DISPLAY_ON | FLAG_DISPLAY_CONTROL_CURSOR_OFF | FLAG_DISPLAY_CONTROL_BLINK_OFF;

        // left-justified left-to-right text
        i2c_lcd1602_info->entry_mode_flags = FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT | FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF;

        i2c_lcd1602_info->init = true;

        // See page 45/46 of HD44780 data sheet for the initialisation procedure.

        // Wait at least 40ms after power rises above 2.7V before sending commands.
        ets_delay_us(DELAY_POWER_ON);

        err = i2c_lcd1602_reset(i2c_lcd1602_info);
    }
    else
    {
        ESP_LOGE(TAG, "i2c_lcd1602_info is NULL");
        err = ESP_FAIL;
    }
    return err;
}

static esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t col, uint8_t row)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        const int row_offsets[] = {0x00, 0x40, 0x14, 0x54};
        if (row > i2c_lcd1602_info->num_rows)
        {
            row = i2c_lcd1602_info->num_rows - 1;
        }
        if (col > i2c_lcd1602_info->num_columns)
        {
            col = i2c_lcd1602_info->num_columns - 1;
        }
        err = _write_command(i2c_lcd1602_info, COMMAND_SET_DDRAM_ADDR | (col + row_offsets[row]));
    }
    return err;
}

static esp_err_t i2c_lcd1602_set_backlight(i2c_lcd1602_info_t *i2c_lcd1602_info, bool enable)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        i2c_lcd1602_info->backlight_flag = _set_or_clear(i2c_lcd1602_info->backlight_flag, enable, FLAG_BACKLIGHT_ON);
        err = _write_to_expander(i2c_lcd1602_info, 0);
    }
    return err;
}

static esp_err_t i2c_lcd1602_set_display(i2c_lcd1602_info_t *i2c_lcd1602_info, bool enable)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        i2c_lcd1602_info->display_control_flags = _set_or_clear(i2c_lcd1602_info->display_control_flags, enable, FLAG_DISPLAY_CONTROL_DISPLAY_ON);
        err = _write_command(i2c_lcd1602_info, COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags);
    }
    return err;
}

static esp_err_t i2c_lcd1602_set_cursor(i2c_lcd1602_info_t *i2c_lcd1602_info, bool enable)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        i2c_lcd1602_info->display_control_flags = _set_or_clear(i2c_lcd1602_info->display_control_flags, enable, FLAG_DISPLAY_CONTROL_CURSOR_ON);
        err = _write_command(i2c_lcd1602_info, COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags);
    }
    return err;
}

static esp_err_t i2c_lcd1602_set_blink(i2c_lcd1602_info_t *i2c_lcd1602_info, bool enable)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        i2c_lcd1602_info->display_control_flags = _set_or_clear(i2c_lcd1602_info->display_control_flags, enable, FLAG_DISPLAY_CONTROL_BLINK_ON);
        err = _write_command(i2c_lcd1602_info, COMMAND_DISPLAY_CONTROL | i2c_lcd1602_info->display_control_flags);
    }
    return err;
}

static esp_err_t i2c_lcd1602_set_left_to_right(i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        i2c_lcd1602_info->entry_mode_flags |= FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT;
        err = _write_command(i2c_lcd1602_info, COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags);
    }
    return err;
}

static esp_err_t i2c_lcd1602_set_right_to_left(i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        i2c_lcd1602_info->entry_mode_flags &= ~FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT;
        err = _write_command(i2c_lcd1602_info, COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags);
    }
    return err;
}

static esp_err_t i2c_lcd1602_set_auto_scroll(i2c_lcd1602_info_t *i2c_lcd1602_info, bool enable)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        i2c_lcd1602_info->entry_mode_flags = _set_or_clear(i2c_lcd1602_info->entry_mode_flags, enable, FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON);
        err = _write_command(i2c_lcd1602_info, COMMAND_ENTRY_MODE_SET | i2c_lcd1602_info->entry_mode_flags);
    }
    return err;
}

static esp_err_t i2c_lcd1602_scroll_display_left(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        // RAM is not changed
        err = _write_command(i2c_lcd1602_info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_DISPLAY | FLAG_SHIFT_MOVE_LEFT);
    }
    return err;
}

static esp_err_t i2c_lcd1602_scroll_display_right(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        // RAM is not changed
        err = _write_command(i2c_lcd1602_info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_DISPLAY | FLAG_SHIFT_MOVE_RIGHT);
    }
    return err;
}

static esp_err_t i2c_lcd1602_move_cursor_left(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        // RAM is not changed. Shift direction is inverted.
        err = _write_command(i2c_lcd1602_info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_CURSOR | FLAG_SHIFT_MOVE_RIGHT);
    }
    return err;
}

static esp_err_t i2c_lcd1602_move_cursor_right(const i2c_lcd1602_info_t *i2c_lcd1602_info)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        // RAM is not changed. Shift direction is inverted.
        err = _write_command(i2c_lcd1602_info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_CURSOR | FLAG_SHIFT_MOVE_LEFT);
    }
    return err;
}

static esp_err_t i2c_lcd1602_define_char(const i2c_lcd1602_info_t *i2c_lcd1602_info, i2c_lcd1602_custom_index_t index, const uint8_t pixelmap[])
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        index &= 0x07; // only the first 8 indexes can be used for custom characters
        err = _write_command(i2c_lcd1602_info, COMMAND_SET_CGRAM_ADDR | (index << 3));
        for (int i = 0; err == ESP_OK && i < 8; ++i)
        {
            err = _write_data(i2c_lcd1602_info, pixelmap[i]);
        }
    }
    return err;
}

static esp_err_t i2c_lcd1602_write_char(const i2c_lcd1602_info_t *i2c_lcd1602_info, uint8_t chr)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        err = _write_data(i2c_lcd1602_info, chr);
    }
    return err;
}

static esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t *i2c_lcd1602_info, const char *string)
{
    esp_err_t err = ESP_FAIL;
    if (_is_init(i2c_lcd1602_info))
    {
        // ESP_LOGI(TAG, "i2c_lcd1602_write_string: %s", string);
        err = ESP_OK;
        for (int i = 0; err == ESP_OK && string[i]; ++i)
        {
            err = _write_data(i2c_lcd1602_info, string[i]);
        }
    }
    return err;
}

static struct
{
    struct arg_int *on_off;
    struct arg_end *end;
} bl_args;

static struct
{
    struct arg_int *on_off;
    struct arg_end *end;
} bi_args;

static struct
{
    struct arg_int *on_off;
    struct arg_end *end;
} cur_args;

static struct
{
    struct arg_int *on_off;
    struct arg_end *end;
} rsv_args;

static struct
{
    struct arg_int *x;
    struct arg_int *y;
    struct arg_end *end;
} move_args;

static struct
{
    struct arg_str *c;
    struct arg_end *end;
} char_args;

static int do_bl_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&bl_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, bl_args.end, argv[0]);
        return 0;
    }

    int on_off = bl_args.on_off->ival[0];

    if (on_off == 0)
    {
        i2c_lcd1602_set_backlight(lcd_info, false);
    }
    else if (on_off == 1)
    {
        i2c_lcd1602_set_backlight(lcd_info, true);
    }
    return 0;
}

static int do_bi_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&bi_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, bi_args.end, argv[0]);
        return 0;
    }

    int on_off = bi_args.on_off->ival[0];

    if (on_off == 0)
    {
        i2c_lcd1602_set_blink(lcd_info, false);
    }
    else if (on_off == 1)
    {
        i2c_lcd1602_set_blink(lcd_info, true);
    }
    return 0;
}

static int do_cur_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&cur_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, cur_args.end, argv[0]);
        return 0;
    }

    int on_off = cur_args.on_off->ival[0];

    if (on_off == 0)
    {
        i2c_lcd1602_set_cursor(lcd_info, false);
    }
    else if (on_off == 1)
    {
        i2c_lcd1602_set_cursor(lcd_info, true);
    }
    return 0;
}

static int do_rsv_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&rsv_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, rsv_args.end, argv[0]);
        return 0;
    }

    int on_off = rsv_args.on_off->ival[0];

    if (on_off == 0)
    {
        i2c_lcd1602_set_left_to_right(lcd_info);
    }
    else if (on_off == 1)
    {
        i2c_lcd1602_set_right_to_left(lcd_info);
    }
    return 0;
}

static int do_move_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&move_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, move_args.end, argv[0]);
        return 0;
    }

    int x = move_args.x->ival[0];
    int y = move_args.y->ival[0];

    if (x < 16 && y < 2)
    {
        i2c_lcd1602_move_cursor(lcd_info, x, y);
    }
    return 0;
}

static int do_char_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&char_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, char_args.end, argv[0]);
        return 0;
    }

    const char *s = char_args.c->sval[0];

    if (strlen(s) == 1)
    {
        i2c_lcd1602_write_char(lcd_info, s[0]);
    }
    else if (strlen(s) < 32)
    {
        i2c_lcd1602_write_string(lcd_info, s);
    }
    return 0;
}

static int do_reset_cmd(int argc, char **argv)
{
    return i2c_lcd1602_reset(lcd_info);
}

static void register_bl(void)
{
    bl_args.on_off = arg_int1("s", "status", "<0|1>", "背光控制");
    bl_args.end = arg_end(1);
    const esp_console_cmd_t bl_cmd = {
        .command = "bl",
        .help = "打开关闭背光(bl -s 0)!",
        .hint = NULL,
        .func = &do_bl_cmd,
        .argtable = &bl_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&bl_cmd));
}

static void register_bi(void)
{
    bi_args.on_off = arg_int1("s", "status", "<0|1>", "闪烁控制");
    bi_args.end = arg_end(1);
    const esp_console_cmd_t bi_cmd = {
        .command = "bi",
        .help = "打开关闭背光(bi -s 0)!",
        .hint = NULL,
        .func = &do_bi_cmd,
        .argtable = &bi_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&bi_cmd));
}

static void register_cur(void)
{
    cur_args.on_off = arg_int1("s", "status", "<0|1>", "光标控制");
    cur_args.end = arg_end(1);
    const esp_console_cmd_t cur_cmd = {
        .command = "cur",
        .help = "打开关闭光标(bi -s 0)!",
        .hint = NULL,
        .func = &do_cur_cmd,
        .argtable = &cur_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cur_cmd));
}

static void register_rsv(void)
{
    rsv_args.on_off = arg_int1("-s", "status", "<0|1>", "反向坐标");
    rsv_args.end = arg_end(1);
    const esp_console_cmd_t bl_rsv = {
        .command = "rsv",
        .help = "打开关闭光标(rsv -s 0)!",
        .hint = NULL,
        .argtable = &rsv_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&bl_rsv));
}

static void register_move(void)
{
    move_args.x = arg_int1("x", "xaxis", "<n>", "X坐标(0-15)!");
    move_args.y = arg_int1("y", "yaxis", "<n>", "Y坐标(0-1)!");
    move_args.end = arg_end(2);
    const esp_console_cmd_t move_cmd = {
        .command = "move",
        .help = "移动坐标(move -x 3 -y 1)!",
        .hint = NULL,
        .func = &do_move_cmd,
        .argtable = &move_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&move_cmd));
}

static void register_char(void)
{
    char_args.c = arg_str1("c", "char", "<string>", "显示字符串");
    char_args.end = arg_end(1);
    const esp_console_cmd_t char_cmd = {
        .command = "char",
        .help = "打印字符或字符串(char -c \"A\" 或者 char -c \"ABC\")!",
        .hint = NULL,
        .func = &do_char_cmd,
        .argtable = &char_args};
    ESP_ERROR_CHECK(esp_console_cmd_register(&char_cmd));
}

static void register_reset(void)
{
    const esp_console_cmd_t bl_cmd = {
        .command = "reset",
        .help = "复位LCD(无参数)",
        .hint = NULL,
        .func = &do_reset_cmd,
        .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&bl_cmd));
}

void register_lcd1602(void)
{
    // Set up the LCD1602 device with backlight off
    lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, I2C_NUM_0, true, LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));
    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));
    i2c_lcd1602_set_backlight(lcd_info, true);
    i2c_lcd1602_set_display(lcd_info, true);

    register_bl();
    register_bi();
    register_cur();
    register_rsv();
    register_reset();
    register_move();
    register_char();
}
