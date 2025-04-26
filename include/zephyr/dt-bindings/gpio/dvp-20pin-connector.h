/*
 * Copyright (c) 2025 tinyVision.ai Inc.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef INCLUDE_ZEPHYR_DT_BINDINGS_GPIO_DVP_20PIN_CONNECTOR_H_
#define INCLUDE_ZEPHYR_DT_BINDINGS_GPIO_DVP_20PIN_CONNECTOR_H_

/** Pin number mask (0..20). */
#define DVP_20PIN_MASK 0xff

/**
 * @name Arducam DVP 20-pin or 18-pin connector pinout
 * @{
 */
#define DVP_20PIN_SCL	3	/**< I2C pin */
#define DVP_20PIN_SDA	4	/**< I2C pin */
#define DVP_20PIN_VS	5	/**< Vertical sync */
#define DVP_20PIN_HS	6	/**< Horizontal sync */
#define DVP_20PIN_PCLK	7	/**< Pixel clock used to transmit the data */
#define DVP_20PIN_XCLK	8	/**< System clock often needed for I2C communication */
#define DVP_20PIN_D7	9	/**< Parallel port data */
#define DVP_20PIN_D6	10	/**< Parallel port data */
#define DVP_20PIN_D5	11	/**< Parallel port data */
#define DVP_20PIN_D4	12	/**< Parallel port data */
#define DVP_20PIN_D3	13	/**< Parallel port data */
#define DVP_20PIN_D2	14	/**< Parallel port data */
#define DVP_20PIN_D1	15	/**< Parallel port data */
#define DVP_20PIN_D0	16	/**< Parallel port data */
#define DVP_20PIN_PEN	17	/**< Power Enable, typicaly shorted with pin 19 */
#define DVP_20PIN_PDN	18	/**< Power Down, typicaly shorted with pin 20 */
/** @} */

#endif /* INCLUDE_ZEPHYR_DT_BINDINGS_GPIO_DVP_20PIN_CONNECTOR_H_ */
